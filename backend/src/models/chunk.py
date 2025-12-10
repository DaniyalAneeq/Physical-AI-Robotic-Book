"""ContentChunk model for textbook content storage in Qdrant."""

from enum import Enum
from pydantic import BaseModel, Field
from typing import Optional


class ContentType(str, Enum):
    """Type of content in a chunk."""
    PARAGRAPH = "paragraph"
    CODE_BLOCK = "code_block"
    HEADING = "heading"


class ContentChunk(BaseModel):
    """A segment of textbook content stored as a vector embedding with metadata.

    Stored in: Qdrant Cloud (vectors + payload)
    """
    id: str = Field(..., description="Unique identifier (UUID)")
    module_id: str = Field(..., description="Module identifier, e.g., 'module-1'")
    chapter_id: str = Field(..., description="Chapter identifier, e.g., 'chapter-3'")
    section_id: str = Field(..., description="Section identifier, e.g., 'section-3.2'")
    paragraph_id: int = Field(..., ge=1, description="Paragraph number within section")
    content_type: ContentType = Field(..., description="Type of content")
    text: str = Field(..., max_length=2000, description="Original text content")
    file_path: str = Field(..., description="Source markdown file path relative to docs/")

    # Vector is stored separately in Qdrant, not in the payload
    # vector: list[float] (1536 dimensions for text-embedding-3-small)


class ChunkPayload(BaseModel):
    """Payload schema for Qdrant storage (without vector)."""
    module_id: str
    chapter_id: str
    section_id: str
    paragraph_id: int
    content_type: str
    text: str
    file_path: str
    chapter_title: Optional[str] = None
    section_title: Optional[str] = None

    @classmethod
    def from_content_chunk(cls, chunk: ContentChunk, chapter_title: str = "", section_title: str = "") -> "ChunkPayload":
        """Create payload from ContentChunk."""
        return cls(
            module_id=chunk.module_id,
            chapter_id=chunk.chapter_id,
            section_id=chunk.section_id,
            paragraph_id=chunk.paragraph_id,
            content_type=chunk.content_type.value,
            text=chunk.text,
            file_path=chunk.file_path,
            chapter_title=chapter_title,
            section_title=section_title,
        )
