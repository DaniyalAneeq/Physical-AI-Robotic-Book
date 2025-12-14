"""OpenAI Agents SDK service for RAG-powered textbook assistant."""

import json
import logging
from typing import AsyncGenerator

from openai import AsyncOpenAI

from app.config import get_settings
from app.tools.retriever import retrieve_textbook_content

settings = get_settings()
logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = AsyncOpenAI(api_key=settings.openai_api_key)


SYSTEM_INSTRUCTIONS = """You are a helpful teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

Your role is to:
- Answer questions about robotics, ROS 2, digital twins, AI integration, and related topics
- Provide accurate information based ONLY on the textbook content
- Cite specific modules, chapters, and sections when answering
- Explain complex concepts in clear, accessible language
- Encourage hands-on learning and experimentation

When answering:
1. Use the retrieve_textbook_content tool to search the textbook for relevant information
2. Base your answers strictly on the retrieved content
3. Include citations in your response (e.g., "According to Module 1, Chapter 2...")
4. If the textbook doesn't cover a topic, politely say so and suggest related topics that are covered

Out-of-scope queries:
- General questions unrelated to the textbook content
- Requests for homework solutions without explanation
- Topics not covered in the textbook (politely redirect)

Be concise, accurate, and educational in your responses.
"""


async def get_agent_response_stream(
    user_message: str,
    conversation_id: str,
    module_filter: str | None = None,
) -> AsyncGenerator[tuple[str, str | dict], None]:
    """
    Stream agent response using OpenAI Chat API with function calling.

    This implementation uses the Chat Completions API with streaming and function calling
    instead of the Agents SDK, as it provides better control over streaming.

    Args:
        user_message: User's input message
        conversation_id: Conversation ID for context
        module_filter: Optional module filter for scoped retrieval

    Yields:
        Tuples of (event_type, data) where event_type is one of:
        - "token": Streaming text token
        - "citation": Citation information from retrieval
        - "error": Error message
    """
    try:
        # Define function schema for retrieval tool
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "retrieve_textbook_content",
                    "description": "Search the textbook content for relevant information",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "The search query to find relevant textbook content",
                            },
                            "module_filter": {
                                "type": "string",
                                "description": "Optional module name to filter results (e.g., 'module-1')",
                            },
                            "limit": {
                                "type": "integer",
                                "description": "Maximum number of results to retrieve (default: 5)",
                                "default": 5,
                            },
                        },
                        "required": ["query"],
                    },
                },
            }
        ]

        # Initialize messages with system instructions
        messages = [
            {"role": "system", "content": SYSTEM_INSTRUCTIONS},
            {"role": "user", "content": user_message},
        ]

        # Make initial API call
        response = await openai_client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            tools=tools,
            tool_choice="auto",
            stream=False,  # First call non-streaming to check for tool calls
        )

        # Check if the model wants to call a function
        if response.choices[0].message.tool_calls:
            # Process function call
            tool_call = response.choices[0].message.tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)

            if function_name == "retrieve_textbook_content":
                # Apply module filter if provided
                if module_filter:
                    function_args["module_filter"] = module_filter

                # Call retrieval tool
                logger.info(f"Calling retriever with args: {function_args}")
                retrieval_result = await retrieve_textbook_content(**function_args)

                # Add function result to messages
                messages.append(response.choices[0].message.model_dump(exclude_unset=True))
                messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "name": function_name,
                        "content": retrieval_result,
                    }
                )

                # Extract citations from retrieval result
                # Parse citation info from formatted results
                if "Module:" in retrieval_result:
                    for line in retrieval_result.split("\n"):
                        if line.startswith("[") and "|" in line:
                            yield ("citation", {"text": line.strip()})

                # Get final response with context from retrieval
                final_response = await openai_client.chat.completions.create(
                    model="gpt-4o",
                    messages=messages,
                    stream=True,
                )

                # Stream final response
                async for chunk in final_response:
                    if chunk.choices[0].delta.content:
                        yield ("token", chunk.choices[0].delta.content)

            else:
                yield ("error", f"Unknown function: {function_name}")

        else:
            # No function call needed, stream direct response
            messages.append(response.choices[0].message.model_dump(exclude_unset=True))

            # Stream the response
            streaming_response = await openai_client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                stream=True,
            )

            async for chunk in streaming_response:
                if chunk.choices[0].delta.content:
                    yield ("token", chunk.choices[0].delta.content)

    except Exception as e:
        logger.error(f"Error in agent response stream: {e}", exc_info=True)
        yield ("error", str(e))
