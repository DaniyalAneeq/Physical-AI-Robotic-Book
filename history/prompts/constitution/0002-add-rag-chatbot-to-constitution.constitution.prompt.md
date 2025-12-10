---
id: "0002"
title: "Add RAG Chatbot to Constitution"
stage: constitution
date: 2025-12-10
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: 001-frontend-redesign
user: system
command: /sp.constitution
labels: ["rag", "chatbot", "constitution", "fastapi", "qdrant"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (constitution document update)
---

## Prompt

Update the Physical AI Humanoid Robotics Textbook constitution to include a new section for Integrated RAG Chatbot Development. The chatbot must be embedded within the published book and meet the following requirements:

1. **RAG Chatbot Overview**
   - Must use Retrieval-Augmented Generation (RAG) to answer questions based on the book content.
   - Must utilize OpenAI Agents/ChatKit SDKs for reasoning and response generation.
   - Must support selective content retrieval: answers should only be based on text explicitly selected by the user.

2. **Technical Architecture**
   - Backend: FastAPI
   - Database: Neon Serverless Postgres for storing embeddings and session data
   - Vector search: Qdrant Cloud Free Tier for content retrieval
   - Integration: Fully embedded in Docusaurus book pages with interactive UI

3. **Constitution Rules for RAG Integration**
   - All chatbot responses must maintain technical accuracy and pedagogical clarity.
   - Ensure secure and efficient queries to the database and vector store.
   - Logging and session management must allow analytics without compromising privacy.
   - Include versioning and dependency specifications for all RAG-related components.
   - Document configuration and setup instructions in `CLAUDE.md` for AI workflow.

4. **Workflow Integration**
   - Include RAG chatbot development as a new artifact in the content workflow:
     - Spec: Define the chatbot scope and feature set.
     - Plan: Outline architecture, integration points, and UI/UX design.
     - Tasks: Break down backend, embedding, retrieval, and frontend implementation tasks.
     - Implementation: Embed in the published book following constitution principles.
     - Review: Test for accuracy, response quality, and security compliance.

5. **Quality Gates**
   - Test queries must produce accurate answers from the selected content.
   - Integration must pass build validation and deployment checks.
   - Accessibility: Ensure chatbot UI is navigable and works with assistive tools.

6. **Governance**
   - All updates must comply with existing constitution principles (accuracy, clarity, consistency, standards).
   - Amend constitution version and update templates where necessary.

Use the same style, structure, and headings as the existing constitution (v1.0.0), maintaining rules, rationale, and governance sections. Output the full constitution.md text with this new RAG Chatbot integration included.

## Response snapshot

Updated the Physical AI & Humanoid Robotics Textbook Constitution from v1.0.0 to v1.1.0 (MINOR version bump).

**Added Sections:**
- VII. RAG Chatbot Architecture - Technical stack (FastAPI, Neon Postgres, Qdrant Cloud, OpenAI SDKs) and core requirements (content fidelity, selective retrieval, citation transparency)
- VIII. RAG Security & Privacy - Query security, session management, privacy compliance, rate limiting
- IX. RAG Versioning & Dependencies - Explicit version pinning requirements for all RAG components
- X. Artifact Sequence - Content workflow integration with RAG-specific artifacts
- XI. Content Quality Gates - Standard textbook quality checks
- XII. RAG Chatbot Quality Gates - Accuracy (95%), precision (0.85), latency (<3s p95), accessibility (WCAG 2.1 AA)
- XV. RAG Governance - Compliance, upgrade testing, migration plans, feedback review

**Templates Checked:**
- plan-template.md - No update required (references constitution dynamically)
- spec-template.md - No update required (generic template)
- tasks-template.md - No update required (generic template)

## Outcome

- Impact: Added comprehensive RAG chatbot governance to constitution enabling structured development of embedded Q&A functionality
- Tests: N/A (constitution document)
- Files: 1 file modified (.specify/memory/constitution.md)
- Next prompts: Create RAG chatbot feature spec (`/sp.specify rag-chatbot`), plan implementation (`/sp.plan`), generate tasks (`/sp.tasks`)
- Reflection: Constitution now provides clear guardrails for RAG development including security, privacy, and quality requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - constitution update
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate constitution compliance during first RAG feature spec creation
