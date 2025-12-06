
from qdrant_client import QdrantClient, models

def create_qdrant_collection():
    client = QdrantClient(host="localhost", port=6333) # Replace with your Qdrant Cloud details

    client.recreate_collection(
        collection_name="physical_ai_book",
        vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
    )
    # Expected metadata fields for each point (payload):
    #   - page_number: int
    #   - chapter: str
    #   - heading: str
    #   - text: str
    #   - chunk_id: str
    print("Qdrant collection 'physical_ai_book' created successfully.")

if __name__ == "__main__":
    create_qdrant_collection()
