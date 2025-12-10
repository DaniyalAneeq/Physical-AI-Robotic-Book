"""Content embedding pipeline for textbook markdown files."""

import asyncio
import hashlib
import re
import sys
from pathlib import Path
from typing import Optional
from uuid import uuid4

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from openai import OpenAI

from src.config import get_settings
from src.models.chunk import ContentChunk, ChunkPayload, ContentType
from src.services.retrieval import retrieval_service

settings = get_settings()

# Initialize OpenAI client
openai_client = OpenAI(api_key=settings.openai_api_key)

# Path to documentation
DOCS_PATH = Path(__file__).parent.parent.parent.parent / "AIdd-book" / "docs"


def parse_markdown_file(file_path: Path) -> list[dict]:
    """Parse a markdown file into chunks with metadata.

    Args:
        file_path: Path to markdown file

    Returns:
        List of chunk dictionaries with text and metadata
    """
    content = file_path.read_text(encoding="utf-8")
    chunks = []

    # Extract module and chapter from path
    # Expected format: docs/module-X/chapter-Y.md or docs/module-X/section/file.md
    rel_path = file_path.relative_to(DOCS_PATH)
    parts = rel_path.parts

    module_id = parts[0] if parts else "unknown"
    chapter_id = parts[1].replace(".md", "") if len(parts) > 1 else "unknown"

    # Parse front matter if present
    front_matter = {}
    if content.startswith("---"):
        end_idx = content.find("---", 3)
        if end_idx > 0:
            fm_content = content[3:end_idx]
            content = content[end_idx + 3:].strip()
            for line in fm_content.strip().split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    front_matter[key.strip()] = value.strip()

    # Get title from front matter or first heading
    title = front_matter.get("title", "")
    if not title:
        title_match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
        if title_match:
            title = title_match.group(1)

    # Split by headings
    sections = re.split(r"(^#{1,3}\s+.+$)", content, flags=re.MULTILINE)

    current_section = title
    current_section_id = "intro"
    paragraph_id = 1

    for i, section in enumerate(sections):
        section = section.strip()
        if not section:
            continue

        # Check if this is a heading
        heading_match = re.match(r"^(#{1,3})\s+(.+)$", section)
        if heading_match:
            level = len(heading_match.group(1))
            current_section = heading_match.group(2)
            current_section_id = re.sub(r"[^a-z0-9]+", "-", current_section.lower()).strip("-")
            paragraph_id = 1

            # Add heading as a chunk
            chunks.append({
                "module_id": module_id,
                "chapter_id": chapter_id,
                "section_id": current_section_id,
                "paragraph_id": paragraph_id,
                "content_type": ContentType.HEADING,
                "text": current_section,
                "file_path": str(rel_path),
                "chapter_title": title,
                "section_title": current_section,
            })
            paragraph_id += 1
            continue

        # Split content into paragraphs and code blocks
        parts_list = re.split(r"(```[\s\S]*?```)", section)

        for part in parts_list:
            part = part.strip()
            if not part:
                continue

            # Check if code block
            if part.startswith("```") and part.endswith("```"):
                chunks.append({
                    "module_id": module_id,
                    "chapter_id": chapter_id,
                    "section_id": current_section_id,
                    "paragraph_id": paragraph_id,
                    "content_type": ContentType.CODE_BLOCK,
                    "text": part,
                    "file_path": str(rel_path),
                    "chapter_title": title,
                    "section_title": current_section,
                })
                paragraph_id += 1
            else:
                # Split into paragraphs (double newline)
                paragraphs = re.split(r"\n\n+", part)
                for para in paragraphs:
                    para = para.strip()
                    if not para or len(para) < 20:  # Skip very short paragraphs
                        continue

                    # Truncate if too long
                    if len(para) > 2000:
                        para = para[:2000] + "..."

                    chunks.append({
                        "module_id": module_id,
                        "chapter_id": chapter_id,
                        "section_id": current_section_id,
                        "paragraph_id": paragraph_id,
                        "content_type": ContentType.PARAGRAPH,
                        "text": para,
                        "file_path": str(rel_path),
                        "chapter_title": title,
                        "section_title": current_section,
                    })
                    paragraph_id += 1

    return chunks


def generate_chunk_id(chunk: dict) -> str:
    """Generate a deterministic ID for a chunk based on content."""
    content = f"{chunk['file_path']}:{chunk['section_id']}:{chunk['paragraph_id']}"
    return hashlib.md5(content.encode()).hexdigest()


def get_embedding(text: str) -> list[float]:
    """Get embedding vector for text using OpenAI API.

    Args:
        text: Text to embed

    Returns:
        Embedding vector (1536 dimensions)
    """
    response = openai_client.embeddings.create(
        model=settings.embedding_model,
        input=text,
    )
    return response.data[0].embedding


async def embed_all_content(force: bool = False):
    """Embed all markdown content and upsert to Qdrant.

    Args:
        force: If True, delete existing content and re-embed
    """
    print(f"Looking for markdown files in: {DOCS_PATH}")

    if not DOCS_PATH.exists():
        print(f"ERROR: Documentation path does not exist: {DOCS_PATH}")
        return

    # Find all markdown files
    md_files = list(DOCS_PATH.glob("**/*.md"))
    md_files = [f for f in md_files if not f.name.startswith("_")]

    print(f"Found {len(md_files)} markdown files")

    if not md_files:
        print("No markdown files found. Exiting.")
        return

    # Initialize Qdrant collection
    print("Ensuring Qdrant collection exists...")
    if force:
        print("Force flag set - deleting existing content...")
        await retrieval_service.delete_all()
    else:
        await retrieval_service.ensure_collection()

    # Process files
    all_chunks = []
    for file_path in md_files:
        print(f"Parsing: {file_path.relative_to(DOCS_PATH)}")
        chunks = parse_markdown_file(file_path)
        all_chunks.extend(chunks)

    print(f"\nTotal chunks extracted: {len(all_chunks)}")

    # Generate embeddings and prepare for upsert
    print("\nGenerating embeddings...")
    batch_size = 50
    processed = 0

    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i:i + batch_size]
        upsert_batch = []

        for chunk in batch:
            chunk_id = generate_chunk_id(chunk)

            # Get embedding
            try:
                vector = get_embedding(chunk["text"])
            except Exception as e:
                print(f"  Error embedding chunk: {e}")
                continue

            # Create payload
            payload = ChunkPayload(
                module_id=chunk["module_id"],
                chapter_id=chunk["chapter_id"],
                section_id=chunk["section_id"],
                paragraph_id=chunk["paragraph_id"],
                content_type=chunk["content_type"].value,
                text=chunk["text"],
                file_path=chunk["file_path"],
                chapter_title=chunk.get("chapter_title", ""),
                section_title=chunk.get("section_title", ""),
            )

            upsert_batch.append((chunk_id, vector, payload))

        # Upsert batch to Qdrant
        if upsert_batch:
            await retrieval_service.upsert_chunks(upsert_batch)

        processed += len(batch)
        print(f"  Processed {processed}/{len(all_chunks)} chunks")

    print("\nEmbedding complete!")

    # Print collection info
    info = await retrieval_service.get_collection_info()
    print(f"\nCollection info: {info}")


def main():
    """Entry point for embedding script."""
    import argparse

    parser = argparse.ArgumentParser(description="Embed textbook content to Qdrant")
    parser.add_argument("--force", action="store_true", help="Delete existing content and re-embed")
    args = parser.parse_args()

    print("=" * 50)
    print("RAG Chatbot Content Embedding")
    print("=" * 50)

    asyncio.run(embed_all_content(force=args.force))


if __name__ == "__main__":
    main()
