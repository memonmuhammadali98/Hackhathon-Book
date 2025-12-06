import re
from typing import List, Dict
import hashlib

def chunk_markdown(text: str, chunk_size: int = 800, overlap: int = 100) -> List[Dict[str, str]]:
    """
    Split markdown text into chunks with overlap.

    Args:
        text: The markdown text to chunk
        chunk_size: Target size of each chunk in characters
        overlap: Number of characters to overlap between chunks

    Returns:
        List of dictionaries containing chunk text and metadata
    """
    # Remove excessive whitespace
    text = re.sub(r'\n\s*\n', '\n\n', text)

    # Split by sections (markdown headers)
    sections = re.split(r'(^#{1,6}\s+.+$)', text, flags=re.MULTILINE)

    chunks = []
    current_chunk = ""
    current_heading = ""

    for i, section in enumerate(sections):
        # Check if this is a heading
        if re.match(r'^#{1,6}\s+', section):
            current_heading = section.strip()

            # If current chunk is too large, save it before adding heading
            if len(current_chunk) > chunk_size:
                if current_chunk.strip():
                    chunks.append({
                        "text": current_chunk.strip(),
                        "heading": current_heading
                    })
                current_chunk = section + "\n\n"
            else:
                current_chunk += section + "\n\n"
        else:
            # This is content
            current_chunk += section

            # If chunk exceeds size, split it
            if len(current_chunk) > chunk_size:
                # Try to split at paragraph boundaries
                paragraphs = current_chunk.split('\n\n')
                temp_chunk = ""

                for para in paragraphs:
                    if len(temp_chunk) + len(para) < chunk_size:
                        temp_chunk += para + "\n\n"
                    else:
                        if temp_chunk.strip():
                            chunks.append({
                                "text": temp_chunk.strip(),
                                "heading": current_heading
                            })
                        temp_chunk = para + "\n\n"

                current_chunk = temp_chunk

    # Add remaining chunk
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "heading": current_heading
        })

    return chunks

def extract_chapter_metadata(filepath: str, content: str) -> Dict[str, str]:
    """
    Extract metadata from markdown file.

    Args:
        filepath: Path to the markdown file
        content: Content of the file

    Returns:
        Dictionary with metadata (chapter, title, etc.)
    """
    metadata = {
        "file_path": filepath,
        "chapter": "",
        "title": ""
    }

    # Extract chapter number from filename or content
    chapter_match = re.search(r'chapter(\d+)', filepath.lower())
    if chapter_match:
        metadata["chapter"] = f"Chapter {chapter_match.group(1)}"

    # Extract title from frontmatter or first heading
    title_match = re.search(r'^title:\s*(.+)$', content, re.MULTILINE)
    if title_match:
        metadata["title"] = title_match.group(1).strip()
    else:
        # Try to find first h1 heading
        h1_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if h1_match:
            metadata["title"] = h1_match.group(1).strip()

    return metadata

def generate_chunk_id(text: str, index: int) -> str:
    """
    Generate a unique ID for a chunk.

    Args:
        text: The chunk text
        index: The chunk index

    Returns:
        A unique chunk ID
    """
    hash_obj = hashlib.md5(text.encode())
    return f"{index}_{hash_obj.hexdigest()[:8]}"

def clean_markdown(text: str) -> str:
    """
    Clean markdown text for better embedding.

    Args:
        text: Raw markdown text

    Returns:
        Cleaned text
    """
    # Remove frontmatter
    text = re.sub(r'^---\n.*?\n---\n', '', text, flags=re.DOTALL)

    # Remove markdown links but keep text
    text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)

    # Remove images
    text = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', '', text)

    # Remove code block markers but keep content
    text = re.sub(r'```[\w]*\n', '\n', text)
    text = re.sub(r'```', '', text)

    # Remove excessive whitespace
    text = re.sub(r'\n\s*\n', '\n\n', text)

    return text.strip()
