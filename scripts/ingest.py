#!/usr/bin/env python3
"""
Document ingestion script for Physical AI Book RAG system.

This script:
1. Walks through the /docs directory
2. Reads all markdown files
3. Chunks them into 512-1024 token segments
4. Embeds using OpenAI
5. Upserts to Qdrant
"""

import os
import sys
from pathlib import Path
import requests
from dotenv import load_dotenv

# Add parent directory to path to import utils
sys.path.append(str(Path(__file__).parent.parent / "rag-backend"))
from utils import chunk_markdown, extract_chapter_metadata, generate_chunk_id, clean_markdown

load_dotenv()

# Configuration
DOCS_DIR = Path(__file__).parent.parent / "physical-ai-book" / "docs"
API_BASE_URL = os.getenv("RAG_API_URL", "http://localhost:8000")

def process_markdown_file(filepath: Path) -> list:
    """Process a single markdown file and return chunks with metadata."""
    print(f"Processing: {filepath}")

    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        # Clean the markdown
        cleaned_content = clean_markdown(content)

        # Extract metadata
        metadata = extract_chapter_metadata(str(filepath), content)

        # Chunk the content
        chunks = chunk_markdown(cleaned_content, chunk_size=800, overlap=100)

        # Add metadata to each chunk
        for i, chunk in enumerate(chunks):
            chunk['chunk_id'] = generate_chunk_id(chunk['text'], i)
            chunk['metadata'] = {
                **metadata,
                'heading': chunk.get('heading', ''),
                'chunk_index': i,
                'total_chunks': len(chunks)
            }

        return chunks

    except Exception as e:
        print(f"Error processing {filepath}: {e}")
        return []

def upsert_chunk(chunk: dict):
    """Upsert a single chunk to the RAG API."""
    try:
        response = requests.post(
            f"{API_BASE_URL}/upsert",
            json={
                "chunk_id": chunk['chunk_id'],
                "text": chunk['text'],
                "metadata": chunk['metadata']
            },
            timeout=30
        )
        response.raise_for_status()
        return True
    except Exception as e:
        print(f"Error upserting chunk {chunk['chunk_id']}: {e}")
        return False

def main():
    """Main ingestion process."""
    print(f"Starting document ingestion from: {DOCS_DIR}")
    print(f"API endpoint: {API_BASE_URL}")

    # Check if API is running
    try:
        health = requests.get(f"{API_BASE_URL}/health", timeout=5)
        health.raise_for_status()
        print("✓ API is healthy")
    except Exception as e:
        print(f"✗ API health check failed: {e}")
        print("Make sure the RAG backend is running (python rag-backend/main.py)")
        sys.exit(1)

    # Find all markdown files
    markdown_files = list(DOCS_DIR.rglob("*.md"))
    print(f"Found {len(markdown_files)} markdown files")

    # Process each file
    total_chunks = 0
    successful_upserts = 0

    for filepath in markdown_files:
        # Skip certain directories
        if any(skip in str(filepath) for skip in ['node_modules', '.docusaurus', 'tutorial-']):
            continue

        chunks = process_markdown_file(filepath)

        for chunk in chunks:
            total_chunks += 1
            if upsert_chunk(chunk):
                successful_upserts += 1
                print(f"✓ Upserted chunk {chunk['chunk_id']}")
            else:
                print(f"✗ Failed to upsert chunk {chunk['chunk_id']}")

    print("\n" + "="*50)
    print(f"Ingestion complete!")
    print(f"Total chunks processed: {total_chunks}")
    print(f"Successful upserts: {successful_upserts}")
    print(f"Failed upserts: {total_chunks - successful_upserts}")
    print("="*50)

if __name__ == "__main__":
    main()
