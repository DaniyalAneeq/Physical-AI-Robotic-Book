# Feature Specification: Internationalization & Localization (English/Urdu)

**Feature Branch**: `002-i18n`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Specify a comprehensive Internationalization (i18n) and Localization architecture for an existing AI-powered book platform to support English and Urdu."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Urdu Interface Navigation (Priority: P1)

A Pakistani engineering student visits the Physical AI & Humanoid Robotics book platform. After logging in, they want to switch the entire interface to Urdu to better understand navigation, menus, and static content in their native language while still accessing the technical content.

**Why this priority**: Core i18n capability. Without UI localization, users cannot effectively navigate the platform in their preferred language. This is the foundation for all other localization features.

**Independent Test**: Can be fully tested by logging in, toggling language to Urdu, and verifying that all navigation menus, buttons, labels, and static pages render in Urdu with proper RTL layout. Delivers immediate value by making the platform accessible to Urdu-speaking users.

**Acceptance Scenarios**:

1. **Given** a logged-in user on the English interface, **When** they click the language toggle and select Urdu, **Then** the UI switches to Urdu locale, all navigation elements display in Urdu, and the layout shifts to RTL
2. **Given** a user viewing the Urdu interface, **When** they navigate between pages, **Then** all static content (About, Contact, Help pages) renders in Urdu with consistent RTL layout
3. **Given** a user on the Urdu interface, **When** they refresh the page, **Then** the Urdu locale and RTL layout persist

---

### User Story 2 - Accessing Pre-translated Book Content (Priority: P2)

A user reading a technical module in English wants to switch to Urdu to better understand the concepts. They expect to see the same chapter structure but with professionally translated Urdu text while preserving technical terminology accuracy.

**Why this priority**: Delivers the core value of bilingual technical education. Enables users to learn complex robotics concepts in their native language while maintaining technical accuracy.

**Independent Test**: Can be tested by navigating to any book module, switching to Urdu, and verifying that the module content displays in Urdu with technical terms either preserved in English or displayed with English equivalents in parentheses (e.g., "Actuator (اییکچوایٹر)").

**Acceptance Scenarios**:

1. **Given** a user viewing Module 1 Chapter 2 in English, **When** they switch to Urdu mode, **Then** the same chapter displays with Urdu content loaded from the database, maintaining the same structure and hierarchy
2. **Given** a user reading Urdu content containing technical terms, **When** they encounter terms like "PID Controller" or "Actuator", **Then** these terms appear either in English or with English equivalents in parentheses
3. **Given** a user in Urdu mode viewing a chapter, **When** the chapter contains code examples, **Then** code blocks remain in English with Urdu comments and explanations surrounding them

---

### User Story 3 - Chatbot Interaction in Urdu (Priority: P3)

A user wants to ask questions about robotics concepts in Urdu using the RAG chatbot. They expect to receive accurate, contextually relevant answers in Urdu that reference the book content appropriately.

**Why this priority**: Enhances user engagement by allowing natural language interaction in their preferred language. Builds on the previous stories to create a fully localized learning experience.

**Independent Test**: Can be tested by opening the chatbot, typing a question in Urdu (e.g., "PID Controller کیا ہے؟"), and verifying that the system returns an accurate answer in Urdu based on English book content embeddings, with technical terms preserved correctly.

**Acceptance Scenarios**:

1. **Given** a user in Urdu mode, **When** they ask a question in Urdu via the chatbot, **Then** the system translates the query to English, retrieves relevant context from English embeddings, and generates an answer in Urdu
2. **Given** a chatbot conversation in Urdu, **When** the answer references specific book sections, **Then** citations and references are formatted in Urdu while preserving technical accuracy
3. **Given** a user asking multiple related questions in Urdu, **When** the conversation continues, **Then** the chatbot maintains context and responds consistently in Urdu throughout the session

---

### User Story 4 - Language Preference Persistence (Priority: P4)

A returning user who previously selected Urdu wants their language preference remembered across sessions. They expect to see the interface and content in Urdu automatically upon login without re-selecting it each time.

**Why this priority**: Quality-of-life improvement that reduces friction for regular users. Not critical for initial launch but important for user retention.

**Independent Test**: Can be tested by logging in, selecting Urdu, logging out, and logging back in to verify that the Urdu locale is automatically applied.

**Acceptance Scenarios**:

1. **Given** a logged-in user who selects Urdu, **When** they log out and log back in, **Then** the interface automatically loads in Urdu
2. **Given** a user with saved Urdu preference, **When** they access the platform from a different device after logging in, **Then** their Urdu preference is applied
3. **Given** a user who changes from Urdu to English, **When** they close and reopen the browser, **Then** the preference update persists

---

### Edge Cases

- What happens when a book module exists in English but has not yet been translated to Urdu? Display English content with a clear indicator that translation is pending.
- How does the system handle mixed-language queries (e.g., "ROS 2 میں navigation کیسے کام کرتا ہے؟")? Detect primary language, translate non-English portions to English for retrieval, respond in detected language.
- What happens when a user switches language mid-chatbot conversation? Maintain conversation context but switch response language; optionally offer to translate previous messages.
- How does the system handle right-to-left layout for complex components like code editors or diagrams? Code blocks remain LTR within RTL layout; diagrams display with RTL-aware captions and labels.
- What happens when Urdu content contains broken or missing translations? Fall back to English content with a visible indicator that translation is incomplete.
- How does the system handle performance when loading large Urdu text with complex Unicode characters? Ensure font loading is optimized; cache rendered text where possible.

## Requirements *(mandatory)*

### Functional Requirements

#### UI & Static Content Localization

- **FR-001**: System MUST provide a language toggle accessible to logged-in users that switches between English and Urdu
- **FR-002**: System MUST apply Urdu locale to all Docusaurus-managed UI elements (navigation, sidebar, footer, buttons, labels) when Urdu is selected
- **FR-003**: System MUST apply RTL (right-to-left) layout styling when Urdu locale is active
- **FR-004**: System MUST persist user language preference in the database linked to their Better Auth user account
- **FR-005**: System MUST load user's saved language preference automatically upon login
- **FR-006**: System MUST provide Urdu translations for all static pages (Home, About, Contact, Help, Privacy Policy)

#### Dynamic Book Content Localization

- **FR-007**: System MUST store Urdu translations of book modules in the database alongside English versions
- **FR-008**: System MUST retrieve and display Urdu book content when a user is in Urdu mode
- **FR-009**: System MUST preserve technical terminology in English or display with English equivalents in parentheses (e.g., "Actuator (اییکچوایٹر)")
- **FR-010**: System MUST maintain identical chapter structure and hierarchy across both languages
- **FR-011**: System MUST preserve code examples in English with Urdu comments and surrounding explanations
- **FR-012**: System MUST display a clear fallback indicator when Urdu translation is not available for a module

#### RAG Chatbot Localization

- **FR-013**: System MUST detect the language of user chatbot queries (English or Urdu)
- **FR-014**: System MUST translate Urdu queries to English before performing vector search on embeddings
- **FR-015**: System MUST retrieve context from English embeddings regardless of query language
- **FR-016**: System MUST instruct the LLM to generate responses in the user's detected query language
- **FR-017**: System MUST preserve technical terms accurately in chatbot responses using the same rules as book content (English or with English equivalents)
- **FR-018**: System MUST cache translated chatbot responses to avoid redundant translation API calls

#### API & Backend Requirements

- **FR-019**: Backend API MUST accept a `locale` header (values: `en`, `ur`) in all content retrieval requests
- **FR-020**: Backend API MUST return content in the requested locale when available
- **FR-021**: Backend API MUST return English content with a `translation_pending` flag when Urdu is unavailable
- **FR-022**: Qdrant vector store MUST include both English and Urdu content fields in each document payload
- **FR-023**: PostgreSQL user table MUST include a `preferred_locale` column (default: `en`)
- **FR-024**: System MUST log language preference changes for analytics

### Key Entities

- **User Locale Preference**: Links a user account to their preferred language (en/ur); stored in PostgreSQL users table as `preferred_locale` column; defaults to English
- **Localized Book Module**: Represents a chapter or section with content in multiple languages; stored in Qdrant with separate fields for English (`content_en`) and Urdu (`content_ur`) text; includes metadata for translation status
- **Translation Cache**: Stores chatbot query-response pairs with language metadata to avoid redundant translations; includes query hash, source language, target language, cached response, and timestamp
- **Locale Context**: Temporary session data indicating current active locale during user interaction; passed via HTTP headers to backend APIs

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can switch from English to Urdu interface in under 2 seconds with complete RTL layout transition
- **SC-002**: 95% of static UI elements (navigation, buttons, labels) display correctly in Urdu without layout breaks
- **SC-003**: Book content retrieval in Urdu mode completes within 200ms (same performance as English)
- **SC-004**: Chatbot responses in Urdu maintain 90%+ accuracy for technical terminology compared to English responses
- **SC-005**: 80% of chatbot responses are served from cache for repeat queries, reducing translation API costs
- **SC-006**: Language preference persists across 100% of user sessions (logout/login, browser refresh, device change)
- **SC-007**: Users report 85%+ satisfaction with Urdu translation quality in technical content (measured via survey)
- **SC-008**: System gracefully handles missing Urdu translations with clear fallback messaging in 100% of cases
- **SC-009**: RTL layout renders correctly across all major browsers (Chrome, Firefox, Safari, Edge) with 95%+ visual consistency
- **SC-010**: Mixed-language queries (Urdu with English technical terms) are correctly processed with 90%+ accuracy

## Assumptions

- Urdu translations of book modules will be created by professional translators before backend implementation; translation workflow is out of scope
- Users must be logged in to access language toggle; anonymous users see English-only content
- English embeddings in Qdrant provide sufficient semantic search quality for Urdu queries after translation
- OpenAI GPT models can accurately translate technical Urdu queries to English and vice versa
- Font support for Urdu Unicode characters is available in modern browsers without custom font loading
- Better Auth session management can store and retrieve locale preference as custom user metadata
- Initial launch supports only English and Urdu; architecture should be extensible to additional languages in future

## Dependencies

- Docusaurus i18n plugin (native to Docusaurus 3.9.2)
- Translation service API (e.g., OpenAI GPT-4 for query/response translation)
- Urdu font rendering support in target browsers (system fonts or web fonts)
- Better Auth user metadata storage capability
- Existing Qdrant vector store schema must support multiple language fields per document
- Existing PostgreSQL schema must allow user table modification for locale preference

## Out of Scope

- Translation of book content (assumes pre-translated Urdu content is provided)
- Support for languages beyond English and Urdu in initial release
- Automatic machine translation of new content as it's added
- Translation management system or CMS for translators
- Voice input/output in Urdu
- Offline support for Urdu content
- Bidirectional synchronization of content updates across languages
- User-generated content translation (comments, forum posts, etc.)

## Risks & Mitigations

- **Risk**: Poor translation quality for technical terms could confuse users
  - **Mitigation**: Implement terminology glossary with approved translations; always show English term alongside Urdu for critical concepts

- **Risk**: RTL layout may break complex UI components (code editors, diagrams, tables)
  - **Mitigation**: Explicitly test all component types; use LTR override for code blocks and technical diagrams; document RTL-safe component patterns

- **Risk**: Translation API costs may exceed budget for high chatbot usage
  - **Mitigation**: Implement aggressive caching strategy; cache all chatbot responses by query hash; monitor costs and implement rate limiting if needed

- **Risk**: Qdrant payload size may increase significantly with dual-language content
  - **Mitigation**: Monitor storage costs; consider separate collections for languages if needed; implement lazy loading for non-active language content

## Architecture Description

### Data Flow Overview

**Translation Pipeline Architecture**:

1. **Ingestion Path** (Pre-translated Content):
   - Content creators provide English and Urdu versions of book modules
   - Ingestion script processes both versions simultaneously
   - English content is embedded using OpenAI embeddings
   - Both English and Urdu text are stored in Qdrant payload fields (`content_en`, `content_ur`)
   - Metadata includes `translation_status: complete|pending|partial`

2. **On-demand Path** (Chatbot Queries):
   - User submits query in Urdu
   - Backend detects language using language detection library
   - Query is translated to English via translation API
   - English query is embedded and searched against Qdrant
   - Retrieved context (English) is passed to LLM with instruction to respond in Urdu
   - Response is cached with query hash for future reuse

**Component Interactions**:

```
[User Browser]
    ↓ (locale header: ur)
[Docusaurus Frontend]
    ↓ (UI: Urdu locale + RTL CSS)
    ↓ (API request with locale header)
[FastAPI Backend]
    ↓ (check locale header)
    ├→ [PostgreSQL] (fetch user locale preference)
    ↓ (retrieve content)
    ├→ [Qdrant] (fetch content_en or content_ur based on locale)
    ↓ (chatbot query path)
    ├→ [Translation API] (translate ur→en for search, en→ur for response)
    └→ [Cache Layer] (store/retrieve translated responses)
```

### API Contract Updates

**Existing Endpoints Modified**:

1. **GET /api/content/{module_id}**
   - **New Header**: `Accept-Language: en|ur` (default: `en`)
   - **Response Changes**:
     - Add `locale` field to response indicating returned language
     - Add `translation_status` field: `complete|pending|partial|unavailable`
     - Return `content` field in requested language or fallback to English

2. **POST /api/chatkit/query**
   - **New Header**: `Accept-Language: en|ur` (default: `en`)
   - **Request Body Changes**: Add optional `query_language` field (auto-detected if not provided)
   - **Response Changes**:
     - Add `response_language` field indicating answer language
     - Add `translation_cached` boolean flag

3. **GET /api/user/preferences**
   - **New Response Field**: `preferred_locale: en|ur`

4. **PUT /api/user/preferences**
   - **New Request Field**: `preferred_locale: en|ur`

### Database Schema Changes

**PostgreSQL (Users Table)**:

```sql
ALTER TABLE users ADD COLUMN preferred_locale VARCHAR(5) DEFAULT 'en';
CREATE INDEX idx_users_locale ON users(preferred_locale);
```

**Qdrant Payload Structure**:

```json
{
  "id": "module-1-chapter-2",
  "content_en": "A PID controller is a control loop mechanism...",
  "content_ur": "PID کنٹرولر ایک کنٹرول لوپ میکانزم ہے...",
  "title_en": "Introduction to PID Controllers",
  "title_ur": "PID کنٹرولرز کا تعارف",
  "module": "module-1",
  "chapter": "chapter-2",
  "translation_status": "complete",
  "technical_terms": ["PID Controller", "Actuator", "Feedback Loop"],
  "embedding": [0.123, 0.456, ...],
  "metadata": {
    "last_updated_en": "2025-12-01",
    "last_updated_ur": "2025-12-10",
    "translator": "professional"
  }
}
```

**Translation Cache (PostgreSQL or Redis)**:

```sql
CREATE TABLE translation_cache (
  id SERIAL PRIMARY KEY,
  query_hash VARCHAR(64) UNIQUE NOT NULL,
  source_language VARCHAR(5) NOT NULL,
  target_language VARCHAR(5) NOT NULL,
  original_query TEXT NOT NULL,
  translated_response TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  access_count INTEGER DEFAULT 1
);

CREATE INDEX idx_translation_cache_hash ON translation_cache(query_hash);
CREATE INDEX idx_translation_cache_languages ON translation_cache(source_language, target_language);
```

### Frontend Implementation Plan

**Docusaurus i18n Configuration** (`docusaurus.config.js`):

```javascript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },
  // ... rest of config
};
```

**RTL CSS Strategy**:

1. Leverage Docusaurus built-in RTL support via `[dir="rtl"]` selectors
2. Override specific components requiring LTR within RTL context:

```css
/* Global RTL support */
[dir="rtl"] {
  text-align: right;
}

/* Code blocks remain LTR */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}

/* Diagrams and images remain centered or LTR-aligned */
[dir="rtl"] .diagram-container {
  direction: ltr;
}

/* Technical terms with English equivalents */
.technical-term {
  font-family: 'Roboto', sans-serif; /* Ensure English renders clearly */
}
```

**Language Toggle Component**:

```typescript
// Location: AIdd-book/src/components/LanguageToggle/index.tsx
// Provides UI toggle that:
// 1. Calls Better Auth API to update user preference
// 2. Triggers Docusaurus locale switch
// 3. Applies RTL stylesheet
```

**Folder Structure for Translations**:

```
AIdd-book/
├── i18n/
│   └── ur/
│       ├── docusaurus-plugin-content-docs/
│       │   └── current/
│       │       └── (static translations if any)
│       └── code.json  (UI translations)
└── src/
    └── components/
        └── LanguageToggle/
```

## Performance Requirements

- **PR-001**: Language toggle must complete UI transition within 2 seconds
- **PR-002**: Content retrieval must not exceed 200ms latency regardless of locale
- **PR-003**: Chatbot query translation must not add more than 500ms to total response time
- **PR-004**: Cache hit rate for translated chatbot responses must exceed 80%
- **PR-005**: RTL layout rendering must not degrade page load time by more than 10%
- **PR-006**: Qdrant payload size must not exceed 150% of English-only size

## Security & Privacy Requirements

- **SPR-001**: Language preference must be stored securely in Better Auth user session
- **SPR-002**: Translation API keys must be stored in environment variables and never exposed to frontend
- **SPR-003**: Cached translations must not leak user-identifiable information
- **SPR-004**: Locale headers must be validated to prevent injection attacks
- **SPR-005**: All user input (queries) must be sanitized before translation

## Acceptance Criteria Summary

1. **UI Translation**: All static elements render correctly in Urdu with RTL layout
2. **Content Retrieval**: Book modules display in Urdu when available, with clear fallback to English
3. **Chatbot Functionality**: Urdu queries return accurate, contextually relevant answers in Urdu
4. **Performance**: All operations meet defined latency limits (toggle < 2s, retrieval < 200ms, chatbot < 500ms overhead)
5. **Terminology Accuracy**: Technical terms preserved in English or displayed with English equivalents in 95%+ of cases
6. **Cache Efficiency**: 80%+ cache hit rate for chatbot translations
7. **Persistence**: Language preference saves and loads correctly across sessions
8. **Graceful Degradation**: Missing Urdu content displays English with clear indicator

## Future Considerations

- Support for additional languages (Arabic, Chinese, Spanish)
- Real-time collaborative translation interface for content creators
- User-contributed translation improvements
- Audio narration in Urdu for accessibility
- Automatic content synchronization when English content updates
