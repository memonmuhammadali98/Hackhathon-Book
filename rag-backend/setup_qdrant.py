
from qdrant_client import QdrantClient, models
import os

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

client = QdrantClient(
    url=QDRANT_HOST,
    api_key=QDRANT_API_KEY,
)

collection_name = "physical_ai_book"

# Create collection if it doesn't exist
if not client.collection_exists(collection_name=collection_name):
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
    )
    print(f"Collection '{collection_name}' created.")
else:
    print(f"Collection '{collection_name}' already exists.")

# Set collection metadata
# Qdrant metadata is per point, not per collection. We will ensure metadata is added during upsert.
print("Qdrant client setup complete.")
