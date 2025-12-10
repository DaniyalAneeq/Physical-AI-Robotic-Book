"""ChatKit server implementation using official OpenAI ChatKit SDK."""

from collections.abc import AsyncIterator
from datetime import datetime
from typing import Any, Literal

from agents import Agent, Runner
from chatkit.server import ChatKitServer
from chatkit.store import Store, Page, default_generate_id, NotFoundError
from chatkit.types import (
    Attachment,
    HiddenContextItem,
    ThreadItemDoneEvent,
    ThreadMetadata,
    ThreadStreamEvent,
    UserMessageItem,
    ThreadItem,
)
from chatkit.agents import AgentContext, stream_agent_response, simple_to_agent_input

from ..config import get_settings
from ..models.chunk import ChunkPayload
from .embedding import embedding_service
from .retrieval import retrieval_service

settings = get_settings()


class InMemoryStore(Store[dict[str, Any]]):
    """Simple in-memory store for ChatKit threads and items."""

    def __init__(self):
        self._threads: dict[str, ThreadMetadata] = {}
        self._items: dict[str, list[ThreadItem]] = {}
        self._attachments: dict[str, Attachment] = {}

    async def load_thread(self, thread_id: str, context: dict[str, Any]) -> ThreadMetadata:
        if thread_id not in self._threads:
            raise NotFoundError(f"Thread {thread_id} not found")
        return self._threads[thread_id]

    async def save_thread(self, thread: ThreadMetadata, context: dict[str, Any]) -> None:
        self._threads[thread.id] = thread
        if thread.id not in self._items:
            self._items[thread.id] = []

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: dict[str, Any],
    ) -> Page[ThreadItem]:
        items = self._items.get(thread_id, [])

        # Handle ordering
        if order == "desc":
            items = list(reversed(items))

        # Handle pagination
        if after:
            try:
                idx = next(i for i, item in enumerate(items) if item.id == after)
                items = items[idx + 1:]
            except StopIteration:
                pass

        # Apply limit
        items = items[:limit]

        has_more = len(self._items.get(thread_id, [])) > len(items)

        return Page(data=items, has_more=has_more)

    async def save_attachment(self, attachment: Attachment, context: dict[str, Any]) -> None:
        self._attachments[attachment.id] = attachment

    async def load_attachment(self, attachment_id: str, context: dict[str, Any]) -> Attachment:
        if attachment_id not in self._attachments:
            raise NotFoundError(f"Attachment {attachment_id} not found")
        return self._attachments[attachment_id]

    async def delete_attachment(self, attachment_id: str, context: dict[str, Any]) -> None:
        if attachment_id in self._attachments:
            del self._attachments[attachment_id]

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: dict[str, Any],
    ) -> Page[ThreadMetadata]:
        threads = list(self._threads.values())

        # Sort by created_at
        threads.sort(key=lambda t: t.created_at, reverse=(order == "desc"))

        # Handle pagination
        if after:
            try:
                idx = next(i for i, t in enumerate(threads) if t.id == after)
                threads = threads[idx + 1:]
            except StopIteration:
                pass

        # Apply limit
        threads = threads[:limit]

        return Page(data=threads, has_more=len(self._threads) > len(threads))

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: dict[str, Any]
    ) -> None:
        if thread_id not in self._items:
            self._items[thread_id] = []
        self._items[thread_id].append(item)

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: dict[str, Any]
    ) -> None:
        if thread_id in self._items:
            for i, existing in enumerate(self._items[thread_id]):
                if existing.id == item.id:
                    self._items[thread_id][i] = item
                    return
        await self.add_thread_item(thread_id, item, context)

    async def load_item(
        self, thread_id: str, item_id: str, context: dict[str, Any]
    ) -> ThreadItem:
        items = self._items.get(thread_id, [])
        for item in items:
            if item.id == item_id:
                return item
        raise NotFoundError(f"Item {item_id} not found in thread {thread_id}")

    async def delete_thread(self, thread_id: str, context: dict[str, Any]) -> None:
        if thread_id in self._threads:
            del self._threads[thread_id]
        if thread_id in self._items:
            del self._items[thread_id]

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: dict[str, Any]
    ) -> None:
        if thread_id in self._items:
            self._items[thread_id] = [
                item for item in self._items[thread_id] if item.id != item_id
            ]


class TextbookChatKitServer(ChatKitServer[dict[str, Any]]):
    """ChatKit server for textbook RAG responses.

    Implements the official OpenAI ChatKit SDK for beautiful UI
    with streaming responses and citations from textbook content.
    """

    def __init__(self):
        """Initialize ChatKit server with in-memory store."""
        self.memory_store = InMemoryStore()
        super().__init__(self.memory_store)

    def _get_base_instructions(self) -> str:
        """Get base instructions for the textbook assistant."""
        return """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Your role is to answer learner questions using ONLY the provided textbook content.

IMPORTANT RULES:
1. Only answer questions based on the provided context from the textbook
2. If the question cannot be answered from the provided context, say "I can only answer questions about the textbook content. This topic doesn't appear to be covered in the current context."
3. Always cite your sources by referring to the chapter and section
4. Be concise but thorough
5. Use markdown formatting for code blocks when showing code examples
6. If asked about topics outside robotics, AI, or the textbook scope, politely redirect

When answering:
- Reference specific chapters and sections
- Quote relevant passages when helpful
- Explain concepts clearly for learners"""

    def _build_context_prompt(self, chunks: list[tuple[ChunkPayload, float]]) -> str:
        """Build context from retrieved chunks.

        Args:
            chunks: Retrieved chunks with relevance scores

        Returns:
            Context string for the model
        """
        if not chunks:
            return "No relevant textbook content found for this query."

        context_parts = []
        for i, (chunk, score) in enumerate(chunks, 1):
            context_parts.append(
                f"[Source {i}] (Chapter: {chunk.chapter_title}, Section: {chunk.section_title})\n"
                f"{chunk.text}"
            )

        return "TEXTBOOK CONTEXT:\n\n" + "\n\n---\n\n".join(context_parts)

    async def _retrieve_context(self, query: str) -> list[tuple[ChunkPayload, float]]:
        """Retrieve relevant context from the textbook.

        Args:
            query: User's question

        Returns:
            List of (chunk, score) tuples
        """
        try:
            query_vector = await embedding_service.embed(query)
            from ..models.session import ScopeContext, ScopeType
            scope = ScopeContext(type=ScopeType.ALL)
            chunks = await retrieval_service.search(
                query_vector=query_vector,
                scope=scope,
                limit=settings.max_chunks_retrieved,
            )
            return chunks
        except Exception as e:
            print(f"Error retrieving context: {e}")
            return []

    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: dict[str, Any],
    ) -> AsyncIterator[ThreadStreamEvent]:
        """Stream ThreadStreamEvent instances for a new user message.

        This is the main method that handles user messages and generates responses
        using the Agents SDK with RAG context.

        Args:
            thread: Metadata for the thread being processed.
            input_user_message: The incoming message the server should respond to.
            context: Arbitrary per-request context.

        Yields:
            Events representing the server's response.
        """
        if not input_user_message:
            return

        # Extract user query from message
        user_query = ""
        if input_user_message.content:
            for content_item in input_user_message.content:
                if hasattr(content_item, 'text'):
                    user_query = content_item.text
                    break

        if not user_query:
            return

        # Retrieve relevant context from textbook
        chunks = await self._retrieve_context(user_query)
        context_prompt = self._build_context_prompt(chunks)

        # Add context as hidden item so model can see it
        hidden_context = HiddenContextItem(
            id=self.store.generate_item_id("message", thread, context),
            thread_id=thread.id,
            created_at=datetime.now(),
            content=[context_prompt],
        )
        yield ThreadItemDoneEvent(item=hidden_context)

        # Create agent context for streaming
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Build the full prompt with context
        full_instructions = f"""{self._get_base_instructions()}

{context_prompt}"""

        # Create agent with context-aware instructions
        contextual_agent = Agent[AgentContext](
            model=settings.chat_model,
            name="Textbook Assistant",
            instructions=full_instructions,
        )

        # Run the agent and stream response
        result = Runner.run_streamed(
            contextual_agent,
            await simple_to_agent_input(input_user_message) if input_user_message else [],
            context=agent_context,
        )

        # Stream the agent response events
        async for event in stream_agent_response(agent_context, result):
            yield event


# Create singleton instance
chatkit_server = TextbookChatKitServer()
