# Quickstart: RAG Chatbot Development

**Feature Branch**: `003-rag-chatbot`
**Date**: 2025-12-10

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus frontend)
- OpenAI API key
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)

## Environment Setup

### 1. Clone and Checkout Branch

```bash
git clone <repository-url>
cd AI-robotics
git checkout 003-rag-chatbot
```

### 2. Backend Setup

```bash
# Create virtual environment
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env
```

### 3. Configure Environment Variables

Edit `backend/.env`:

```env
# OpenAI API
OPENAI_API_KEY=sk-your-api-key-here

# Qdrant Cloud
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Server Config
HOST=0.0.0.0
PORT=8000
DEBUG=true
```

### 4. Database Setup

```bash
# Run migrations (from backend directory)
python -m src.scripts.migrate

# This creates:
# - sessions table
# - query_logs table
# - Required indexes
```

### 5. Initialize Vector Store

```bash
# Create Qdrant collection and embed textbook content
python -m src.scripts.embed_content

# This:
# - Parses markdown files from AIdd-book/docs/
# - Generates embeddings via OpenAI
# - Upserts to Qdrant with metadata
```

### 6. Run Backend Server

```bash
# Development server with hot reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Verify health check
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "dependencies": {
    "qdrant": "connected",
    "postgres": "connected",
    "openai": "connected"
  }
}
```

### 7. Frontend Setup

```bash
# From repository root
cd AIdd-book

# Install dependencies
npm install

# Start development server
npm run start
```

The chatbot component will be available on any documentation page.

## Quick Verification

### Test Chat API

```bash
# Create a session
curl -X POST http://localhost:8000/api/sessions \
  -H "Content-Type: application/json"

# Response: {"id": "session-uuid", ...}

# Send a query
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "session-uuid-from-above",
    "query": "What is ROS 2?",
    "scope": {"type": "all"}
  }'
```

### Test Frontend

1. Open http://localhost:3000
2. Navigate to any documentation page
3. Click the chatbot toggle button (bottom-right)
4. Type a question about the textbook content
5. Verify response includes citations

## Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app
│   ├── config.py            # Environment config
│   ├── models/              # Pydantic models
│   ├── services/            # Business logic
│   ├── api/                 # Routes and middleware
│   └── scripts/             # CLI tools
├── tests/                   # Test suite
├── requirements.txt
└── .env

AIdd-book/
├── src/
│   ├── components/          # React components
│   │   ├── ChatbotPanel/    # Main chat UI
│   │   └── ChatbotToggle/   # Toggle button
│   ├── hooks/               # Custom hooks
│   ├── services/            # API client
│   └── theme/               # Swizzled components
└── docusaurus.config.ts
```

## Common Tasks

### Re-embed Content

After updating textbook content:

```bash
python -m src.scripts.embed_content --force
```

### Clear Sessions

For testing or cleanup:

```bash
python -m src.scripts.clear_sessions
```

### Run Tests

```bash
# Backend tests
cd backend
pytest

# Frontend tests
cd AIdd-book
npm run test
```

### Build for Production

```bash
# Backend (Docker)
cd backend
docker build -t rag-chatbot-api .

# Frontend (Docusaurus)
cd AIdd-book
npm run build
```

## Troubleshooting

### "Connection refused" to Qdrant

- Verify `QDRANT_URL` includes `https://`
- Check API key is correct
- Ensure cluster is active in Qdrant Cloud dashboard

### "OpenAI rate limit" errors

- Check API key has sufficient quota
- Reduce batch size in embedding script
- Add delays between requests

### Frontend not connecting to backend

- Ensure backend is running on port 8000
- Check CORS configuration in `backend/src/api/middleware.py`
- Verify `chatApi.ts` points to correct URL

### Session expiring too quickly

- Check `NEON_DATABASE_URL` is correct
- Verify session cleanup job isn't running too frequently
- Check server time synchronization

## Next Steps

1. Review [spec.md](./spec.md) for full requirements
2. Review [plan.md](./plan.md) for architecture details
3. Review [data-model.md](./data-model.md) for entity schemas
4. Run `/sp.tasks` to generate implementation tasks
