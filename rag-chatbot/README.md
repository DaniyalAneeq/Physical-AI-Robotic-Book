# RAG Chatbot - Physical AI & Humanoid Robotics Textbook Assistant

A production-ready RAG (Retrieval Augmented Generation) chatbot that provides intelligent question-answering capabilities for the "Physical AI & Humanoid Robotics" textbook.

## Architecture

- **Backend**: FastAPI + OpenAI Chat API with function calling
- **Frontend**: React component for Docusaurus
- **Vector Database**: Qdrant Cloud (1536-dimensional embeddings)
- **Relational Database**: Neon Serverless PostgreSQL
- **Embeddings**: OpenAI `text-embedding-3-small`
- **LLM**: OpenAI `gpt-4o` with streaming

## Features

### ✅ User Story 1: Ask Questions About Textbook Content
- Semantic search over textbook markdown files
- Accurate, grounded answers with citations
- Module/chapter/section attribution
- <2s p95 latency

### ✅ User Story 2: Multi-Turn Conversations
- Context-aware follow-up questions
- Conversation persistence in PostgreSQL
- Session management via localStorage UUID

### ✅ User Story 3: Real-Time Streaming Responses
- Server-Sent Events (SSE) streaming
- <1s first token latency
- Word-by-word response display
- Graceful error handling

### ✅ User Story 4: Access Conversation History
- List all conversations per session
- Load previous conversations
- Rename and delete conversations
- 7-day retention policy

### ✅ User Story 5: Module-Specific Queries
- Filter queries by module (e.g., "module-1")
- Qdrant payload filtering
- Optional "All Modules" mode

## Quick Start

### Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus frontend)
- [UV package manager](https://github.com/astral-sh/uv)
- OpenAI API key
- Qdrant Cloud account (free tier)
- Neon PostgreSQL database (free tier)

### 1. Backend Setup

```bash
cd rag-chatbot/backend

# Install dependencies with UV
uv sync

# Configure environment variables
cp .env.example .env
# Edit .env with your API keys

# Run database migrations
uv run alembic upgrade head

# Initialize Qdrant collection
uv run python scripts/init_qdrant.py

# Index textbook content
uv run python scripts/index_textbook.py

# Start development server
uv run uvicorn app.main:app --reload --port 8000
```

### 2. Frontend Integration

The TextbookChat component is located at:
```
AIdd-book/src/components/TextbookChat/
```

To use it in a Docusaurus page:

```tsx
import TextbookChat from '@site/src/components/TextbookChat';

<TextbookChat apiUrl="http://localhost:8000/chatkit" />
```

### 3. Environment Variables

**Backend** (`.env`):
```bash
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.us-east4-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=...
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
```

**Frontend** (`.env.example`):
```bash
CHATKIT_API_URL=http://localhost:8000/chatkit
```

## API Endpoints

### Health Check
```
GET /health
```

### ChatKit Streaming
```
POST /chatkit
Content-Type: application/json

{
  "message": "What is ROS 2?",
  "session_id": "uuid-from-browser",
  "conversation_id": "uuid-or-null",
  "context": {
    "module_filter": "module-1"  // optional
  }
}
```

### Conversation Management
```
GET /api/conversations?session_id=<uuid>
GET /api/conversations/<id>
PATCH /api/conversations/<id>?title=New+Title
DELETE /api/conversations/<id>
```

### Index Management
```
GET /api/index/status
POST /api/index/rebuild
```

## Database Schema

### PostgreSQL (Neon)

- **conversations**: User chat sessions
- **messages**: Individual messages (user/assistant)
- **query_logs**: Analytics and debugging

### Qdrant

- **textbook_chunks**: Vector embeddings with payload:
  - `text`: Chunk content
  - `module`: Module identifier
  - `chapter`: Chapter filename
  - `section`: Section heading
  - `file_path`: Relative path from docs/

## Maintenance

### Cleanup Job (Cron/Scheduler)

Run daily to remove expired data:

```bash
uv run python scripts/cleanup.py
```

Deletes:
- Conversations older than 7 days
- Query logs older than 30 days

### Re-indexing

To rebuild the vector index after textbook updates:

```bash
uv run python scripts/index_textbook.py
```

## Development

### Code Quality

```bash
# Lint code
uv run ruff check .

# Format code
uv run ruff format .

# Type check (optional)
uv run mypy app/
```

### Testing

```bash
# Run tests
uv run pytest

# With coverage
uv run pytest --cov=app --cov-report=html
```

## Deployment

### Backend (Docker)

```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY . .
RUN pip install uv && uv sync
CMD ["uv", "run", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Frontend (Vercel/Netlify)

Docusaurus builds to static files. Deploy via:
```bash
npm run build
# Deploy build/ directory
```

## Troubleshooting

### "Collection not found" (Qdrant)
```bash
uv run python scripts/init_qdrant.py
```

### "Connection refused" (Neon)
- Verify `DATABASE_URL` includes `?sslmode=require`
- Check Neon dashboard for compute status

### CORS errors
- Add frontend URL to `CORS_ORIGINS` in `.env`
- Restart backend server

### Slow responses
- Check OpenAI API rate limits
- Verify Qdrant collection has indexed chunks
- Review `query_logs` table for latency metrics

## Architecture Decisions

See `/specs/004-rag-chatbot-rebuild/plan.md` for detailed technical decisions including:
- OpenAI Chat API with function calling (instead of Agents SDK for better streaming control)
- Qdrant payload filtering for module-specific queries
- Anonymous session management via localStorage UUID
- Circuit breaker pattern for external service resilience

## License

Part of the "Physical AI & Humanoid Robotics" textbook project.
