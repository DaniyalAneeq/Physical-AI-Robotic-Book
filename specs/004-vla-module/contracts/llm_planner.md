# API Contract: LLM Planner

This document describes the interaction with a generic Large Language Model (LLM) for cognitive planning.

- **Endpoint**: (Varies by provider, e.g., `https://api.openai.com/v1/chat/completions`)
- **Method**: `POST`
- **Authentication**: Bearer Token

## Request Body (Example for OpenAI)

- `model`: The LLM model to use (e.g., "gpt-4").
- `messages`: A list of messages, including the system prompt and user prompt.
  - **System Prompt**: Defines the role of the LLM as a robot planner and the desired output format (e.g., JSON).
  - **User Prompt**: Contains the natural language command and the current state of the environment.

## Success Response (200 OK)

- **Content-Type**: `application/json`
- **Body** (Example for OpenAI):
  ```json
  {
    "choices": [
      {
        "message": {
          "role": "assistant",
          "content": "{\"actions\": [{\"name\": \"navigate_to_location\", \"parameters\": {\"location\": \"kitchen\"}}]}"
        }
      }
    ]
  }
  ```
  The `content` will be a JSON string representing the `ActionPlan`.

## Error Response

- Standard LLM provider error format.
