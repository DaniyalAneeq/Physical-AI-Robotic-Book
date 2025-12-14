"""Markdown document chunking service for textbook content."""

import hashlib
import re
from pathlib import Path


def chunk_markdown(content: str, file_path: str) -> list[dict]:
    """
    Chunk markdown content by headings with metadata extraction.

    Splits markdown documents at heading boundaries (##, ###, ####) to preserve
    semantic coherence. Each chunk includes metadata for filtering and citation.

    Args:
        content: Raw markdown content
        file_path: Relative file path from docs/ root (e.g., "module-1/01-intro.md")

    Returns:
        List of chunk dictionaries with text, module, chapter, section, and file_path
    """
    chunks = []

    # Split by headings level 2-4 (##, ###, ####)
    sections = re.split(r"(?=^#{2,4}\s)", content, flags=re.MULTILINE)

    # Extract module and chapter from file path
    path_parts = file_path.split("/")
    module = next((p for p in path_parts if p.startswith("module-")), None)
    chapter = path_parts[-1].replace(".md", "") if path_parts else "unknown"

    for section in sections:
        if not section.strip():
            continue

        # Extract section heading
        heading_match = re.match(r"^(#{2,4})\s+(.+)$", section, re.MULTILINE)
        heading = heading_match.group(2).strip() if heading_match else "Introduction"

        # Truncate to ~1500 tokens (~6000 chars max)
        text = section.strip()[:6000]

        # Generate deterministic chunk ID from file path + section
        chunk_id = hashlib.md5(f"{file_path}:{heading}".encode()).hexdigest()

        chunks.append(
            {
                "id": chunk_id,
                "text": text,
                "module": module,
                "chapter": chapter,
                "section": heading,
                "file_path": file_path,
                "char_count": len(text),
            }
        )

    return chunks


def chunk_file(file_path: Path, docs_root: Path) -> list[dict]:
    """
    Read and chunk a single markdown file.

    Args:
        file_path: Absolute path to markdown file
        docs_root: Absolute path to docs/ root directory

    Returns:
        List of chunks with metadata
    """
    content = file_path.read_text(encoding="utf-8")
    relative_path = str(file_path.relative_to(docs_root))
    return chunk_markdown(content, relative_path)


def chunk_directory(docs_path: Path) -> list[dict]:
    """
    Recursively chunk all markdown files in a directory.

    Args:
        docs_path: Path to docs/ directory containing textbook content

    Returns:
        Flat list of all chunks from all files
    """
    all_chunks = []

    for md_file in docs_path.rglob("*.md"):
        # Skip README and other meta files
        if md_file.name.lower() in ("readme.md", "index.md"):
            continue

        chunks = chunk_file(md_file, docs_path)
        all_chunks.extend(chunks)

    return all_chunks
