# Quickstart: RAG Chatbot Full Rebuild

**Feature**: 004-rag-chatbot-rebuild
**Date**: 2025-12-12

---

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend/Docusaurus)
- [UV package manager](https://github.com/astral-sh/uv) installed
- Docker (optional, for local Qdrant)
- Git

### External Services (obtain credentials before starting)

| Service | Purpose | Free Tier |
|---------|---------|-----------|
| [OpenAI API](https://platform.openai.com/) | Embeddings + Chat | Pay-as-you-go |
| [Qdrant Cloud](https://cloud.qdrant.io/) | Vector database | 1GB free |
| [Neon](https://neon.tech/) | PostgreSQL | Free tier available |

---

## Step 1: Clone and Setup Backend

```bash
# Clone repository (if not already)
cd AI-robotics

# Create backend directory structure
mkdir -p rag-chatbot/backend
cd rag-chatbot/backend

# Initialize with UV
uv init .

# Add production dependencies
uv add fastapi uvicorn[standard] openai-agents chatkit-python \
    qdrant-client sqlalchemy alembic psycopg[binary] \
    python-dotenv pydantic tenacity

# Add development dependencies
uv add --dev pytest pytest-asyncio httpx ruff mypy

# Create source structure
mkdir -p app/{api,services,models,tools}
touch app/__init__.py app/main.py app/config.py
touch app/api/__init__.py app/services/__init__.py
touch app/models/__init__.py app/tools/__init__.py
```

---

## Step 2: Environment Configuration

Create `.env` file in `rag-chatbot/backend/`:

```bash
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://xxx.us-east4-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=...

# Neon PostgreSQL
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
```

---

## Step 3: Initialize Database (Alembic)

```bash
# Initialize Alembic
uv run alembic init alembic

# Edit alembic/env.py to use DATABASE_URL from environment
# Edit alembic.ini: sqlalchemy.url = (leave empty, set in env.py)

# Create initial migration
uv run alembic revision --autogenerate -m "initial schema"

# Apply migration
uv run alembic upgrade head
```

---

## Step 4: Initialize Qdrant Collection

```python
# scripts/init_qdrant.py
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
import os

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Create collection for textbook chunks
client.recreate_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
)

# Create payload indexes for filtering
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="module",
    field_schema="keyword",
)
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="chapter",
    field_schema="keyword",
)

print("Qdrant collection 'textbook_chunks' initialized.")
```

Run:
```bash
uv run python scripts/init_qdrant.py
```

---

## Step 5: Index Textbook Content

```python
# scripts/index_textbook.py
import os
import re
import hashlib
from pathlib import Path
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct

DOCS_PATH = Path("../../AIdd-book/docs")
openai = OpenAI()
qdrant = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))

def chunk_markdown(content: str, file_path: str):
    """Split markdown by headings with metadata."""
    chunks = []
    sections = re.split(r'(?=^#{2,4}\s)', content, flags=re.MULTILINE)

    for section in sections:
        if not section.strip():
            continue
        heading_match = re.match(r'^(#{2,4})\s+(.+)$', section, re.MULTILINE)
        heading = heading_match.group(2) if heading_match else "Introduction"

        path_parts = file_path.split('/')
        module = next((p for p in path_parts if p.startswith('module-')), None)
        chapter = path_parts[-1].replace('.md', '')

        chunks.append({
            "text": section.strip()[:6000],  # Max ~1500 tokens
            "module": module,
            "chapter": chapter,
            "section": heading,
            "file_path": file_path,
        })
    return chunks

def embed_text(text: str) -> list[float]:
    response = openai.embeddings.create(
        model="text-embedding-3-small",
        input=text,
    )
    return response.data[0].embedding

def index_all():
    points = []
    for md_file in DOCS_PATH.rglob("*.md"):
        rel_path = str(md_file.relative_to(DOCS_PATH))
        content = md_file.read_text(encoding="utf-8")

        for chunk in chunk_markdown(content, rel_path):
            chunk_id = hashlib.md5(f"{rel_path}:{chunk['section']}".encode()).hexdigest()
            vector = embed_text(chunk["text"])

            points.append(PointStruct(
                id=chunk_id,
                vector=vector,
                payload=chunk,
            ))

    # Batch upsert
    qdrant.upsert(collection_name="textbook_chunks", points=points, wait=True)
    print(f"Indexed {len(points)} chunks.")

if __name__ == "__main__":
    index_all()
```

Run:
```bash
uv run python scripts/index_textbook.py
```

---

## Step 6: Run Development Server

```bash
# Start FastAPI server
uv run uvicorn app.main:app --reload --port 8000

# In another terminal, verify health
curl http://localhost:8000/health
```

Expected response:
```json
{"status": "healthy", "timestamp": "2025-12-12T..."}
```

---

## Step 7: Frontend Integration (Docusaurus)

In `AIdd-book/`:

```bash
# Install ChatKit React
npm install @openai/chatkit-react

# Create component
mkdir -p src/components/TextbookChat
```

Create `src/components/TextbookChat/index.tsx`:
```typescript
import React from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';

export default function TextbookChat(): JSX.Element {
  const { control } = useChatKit({
    api: {
      url: process.env.CHATKIT_API_URL || 'http://localhost:8000/chatkit',
      domainKey: 'textbook-chat',
    },
    theme: { colorScheme: 'light' },
    history: { enabled: true },
    startScreen: {
      greeting: 'Ask me anything about Physical AI & Robotics!',
      prompts: [
        { label: 'What is ROS 2?', prompt: 'Explain ROS 2 and its key concepts' },
        { label: 'Digital Twin basics', prompt: 'What is a digital twin in robotics?' },
      ],
    },
  });

  return <ChatKit control={control} className="h-full w-full" />;
}
```

---

## Step 8: Verify End-to-End

1. Backend running on `http://localhost:8000`
2. Docusaurus running on `http://localhost:3000`
3. Send test query via ChatKit widget
4. Verify response includes citations from textbook content

---

## Development Commands

| Command | Description |
|---------|-------------|
| `uv run uvicorn app.main:app --reload` | Start dev server |
| `uv run pytest` | Run tests |
| `uv run ruff check .` | Lint code |
| `uv run alembic upgrade head` | Apply migrations |
| `uv run python scripts/index_textbook.py` | Re-index content |

---

## Troubleshooting

### "Collection not found" (Qdrant)
Run `scripts/init_qdrant.py` to create the collection.

### "Connection refused" (Neon)
- Verify `DATABASE_URL` format includes `?sslmode=require`
- Check Neon dashboard for connection pooling status

### "Invalid API key" (OpenAI)
- Ensure `OPENAI_API_KEY` starts with `sk-`
- Check API key has embeddings and chat permissions

### CORS errors
- Add frontend URL to `CORS_ORIGINS` in `.env`
- Restart backend server after changes
