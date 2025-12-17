---
id: 0002
title: Auth Refactor Implementation Plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-auth-refactor
branch: 001-auth-refactor
user: developer
command: /sp.plan
labels: ["authentication", "backend-refactor", "onboarding", "fastapi", "better-auth", "planning"]
links:
  spec: specs/001-auth-refactor/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-auth-refactor/plan.md
 - specs/001-auth-refactor/research.md
 - specs/001-auth-refactor/data-model.md
 - specs/001-auth-refactor/contracts/auth-api.yaml
 - specs/001-auth-refactor/contracts/onboarding-api.yaml
 - specs/001-auth-refactor/quickstart.md
tests:
 - Constitution Check validation (all gates passed)
 - Architecture design validation
---

## Prompt

Execute the planning workflow for the authentication refactor feature (001-auth-refactor). Based on the specification in specs/001-auth-refactor/spec.md, generate comprehensive planning artifacts including research, data model, API contracts, and quickstart guide.

## Response snapshot

Generated complete implementation plan with 6 artifacts:

**Phase 0 - Research (research.md)**:
- Researched FastAPI multi-module integration patterns
- Investigated Better Auth best practices (discovered it's a custom Python implementation following Better Auth patterns)
- Analyzed authentication middleware approaches (FastAPI dependency injection vs global middleware)
- Designed OAuth configuration for single-port architecture
- Defined database schema extensions for onboarding
- Documented session management patterns with HTTP-only cookies
- Created onboarding flow implementation strategy

**Phase 1 - Design & Contracts**:

1. **Data Model (data-model.md)**:
   - Defined 5 entities: User (modified), OnboardingProfile (new), Session, OAuthAccount, Conversation
   - Created database migration 002_onboarding (add onboarding_completed flag, create onboarding_profiles table)
   - Documented predefined enumerations (user types, areas of interest, experience levels, topics)
   - Defined common query patterns and validation rules

2. **API Contracts**:
   - **auth-api.yaml**: 10 authentication endpoints (register, login, logout, me, refresh, OAuth flows)
   - **onboarding-api.yaml**: 4 onboarding endpoints (status, options, complete, profile)
   - Full OpenAPI 3.1 specifications with request/response schemas, security schemes, error responses

3. **Quickstart Guide (quickstart.md)**:
   - Prerequisites and environment setup instructions
   - OAuth provider configuration (Google, GitHub)
   - Database migration steps
   - Development workflow and testing procedures
   - Troubleshooting common issues

4. **Project Structure**:
   - Documented target directory structure (rag-chatbot/backend + rag-chatbot/auth_backend)
   - Clarified unified server architecture (single FastAPI app on port 8000)
   - Identified files to create vs modify

5. **Constitution Check**:
   - Initial validation: All requirements align with constitution
   - Post-design validation: All gates passed (authentication, RAG, backend-first, Better Auth patterns)

## Outcome

- ✅ Impact: Created comprehensive implementation plan enabling task breakdown and development. Plan provides clear technical direction for unifying auth and RAG backends, implementing onboarding flow, and maintaining constitution compliance.
- 🧪 Tests: All constitution quality gates passed (authentication security, RAG preservation, backend-first architecture, Better Auth patterns)
- 📁 Files: 6 planning artifacts generated (plan.md, research.md, data-model.md, auth-api.yaml, onboarding-api.yaml, quickstart.md)
- 🔁 Next prompts: Ready for `/sp.tasks` to generate testable implementation tasks
- 🧠 Reflection: Planning phase successfully identified that "Better Auth" is actually a custom Python implementation following Better Auth conceptual patterns (Better Auth itself is TypeScript/JavaScript only). This clarity prevents confusion during implementation. Architecture design for unified server with FastAPI routers is sound and aligns with all constraints.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning artifacts generated successfully
- Graders run and results (PASS/FAIL): Constitution Check PASSED (both initial and post-design), Architecture validation PASSED
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Proceed to task generation phase to validate plan granularity and implementation approach
