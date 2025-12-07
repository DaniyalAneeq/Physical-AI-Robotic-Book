# API Contract: OpenAI Whisper

This document describes the interaction with the OpenAI Whisper API for speech-to-text transcription.

- **Endpoint**: `https://api.openai.com/v1/audio/transcriptions`
- **Method**: `POST`
- **Authentication**: Bearer Token (`Authorization: Bearer $OPENAI_API_KEY`)
- **Content-Type**: `multipart/form-data`

## Request Body

- `file`: The audio file to transcribe.
- `model`: The name of the Whisper model to use (e.g., "whisper-1").

## Success Response (200 OK)

- **Content-Type**: `application/json`
- **Body**:
  ```json
  {
    "text": "The transcribed text."
  }
  ```

## Error Response

- Standard OpenAI API error format.
