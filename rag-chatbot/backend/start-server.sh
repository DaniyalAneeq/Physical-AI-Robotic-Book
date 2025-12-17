#!/bin/bash

# Script to start the RAG Chatbot backend server with authentication enabled

# Navigate to backend directory
cd "$(dirname "$0")"

# Check if .env file exists
if [ ! -f ".env" ]; then
    echo "Error: .env file not found!"
    echo "Please create .env file from .env.example and configure your settings."
    exit 1
fi

# Activate virtual environment and start server
echo "Starting RAG Chatbot backend server..."
echo "Server will be available at: http://localhost:8000"
echo "API documentation: http://localhost:8000/docs"
echo ""

# Start uvicorn with the virtual environment python
.venv/bin/python -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
