"""
Translation service for multilingual chatbot support.

Provides language detection and translation between English and Urdu,
with special handling for technical term preservation.
"""

import os
import re
from typing import List, Optional, Tuple
from langdetect import detect, LangDetectException
import openai
from openai import OpenAI

# Lazy-initialize OpenAI client to avoid errors on module import
_client = None

def get_openai_client():
    """Get or initialize the OpenAI client."""
    global _client
    if _client is None:
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is not set")
        _client = OpenAI(api_key=api_key)
    return _client

# Common technical terms to preserve in translations
TECHNICAL_TERMS = [
    "ROS 2", "ROS2", "Gazebo", "URDF", "LIDAR", "IMU", "PID", "PID Controller",
    "Actuator", "Sensor", "Transformer", "Neural Network", "Deep Learning",
    "Computer Vision", "SLAM", "Navigation", "MoveIt", "Rviz", "Ubuntu",
    "Docker", "Python", "C++", "API", "SDK", "JSON", "XML", "YAML",
    "GitHub", "Git", "MQTT", "HTTP", "TCP/IP", "UDP", "WiFi", "Bluetooth",
    "Arduino", "Raspberry Pi", "NVIDIA", "Isaac Sim", "Unity", "Unreal Engine",
]

# Compile regex pattern for technical terms (case-insensitive)
TECH_TERMS_PATTERN = re.compile(
    r"\b(" + "|".join(re.escape(term) for term in TECHNICAL_TERMS) + r")\b",
    re.IGNORECASE
)


def detect_language(text: str, confidence_threshold: float = 0.8) -> Tuple[str, float]:
    """
    Detect the language of the input text.

    Args:
        text: Input text to detect language from
        confidence_threshold: Minimum confidence required (0.0-1.0)

    Returns:
        Tuple of (language_code, confidence)
        - language_code: 'en', 'ur', or 'unknown'
        - confidence: Detection confidence (0.0-1.0)

    Example:
        >>> detect_language("What is a PID controller?")
        ('en', 0.99)
        >>> detect_language("PID کیا ہے؟")
        ('ur', 0.95)
    """
    # Clean text for detection (remove URLs, special chars)
    cleaned_text = re.sub(r'http\S+|www.\S+', '', text)
    cleaned_text = re.sub(r'[^\w\s\u0600-\u06FF]', ' ', cleaned_text)
    cleaned_text = cleaned_text.strip()

    if not cleaned_text:
        return ("unknown", 0.0)

    try:
        # langdetect returns ISO 639-1 codes
        detected_lang = detect(cleaned_text)

        # Map to our supported languages
        if detected_lang == "en":
            return ("en", 0.99)  # langdetect doesn't provide confidence, use high value
        elif detected_lang == "ur":
            return ("ur", 0.95)
        else:
            # Fallback to English for unsupported languages
            return ("en", 0.50)

    except LangDetectException:
        # Detection failed, default to English
        return ("unknown", 0.0)


def extract_technical_terms(text: str) -> List[str]:
    """
    Extract technical terms from text that should be preserved during translation.

    Args:
        text: Input text to extract terms from

    Returns:
        List of technical terms found in the text

    Example:
        >>> extract_technical_terms("ROS 2 and PID Controller are important")
        ['ROS 2', 'PID Controller']
    """
    matches = TECH_TERMS_PATTERN.findall(text)
    # Return unique terms while preserving order
    seen = set()
    unique_terms = []
    for term in matches:
        if term.lower() not in seen:
            seen.add(term.lower())
            unique_terms.append(term)
    return unique_terms


def translate_urdu_to_english(urdu_query: str) -> str:
    """
    Translate Urdu query to English for vector search.

    Uses OpenAI API to translate while preserving technical terms.

    Args:
        urdu_query: Query in Urdu

    Returns:
        Translated query in English

    Raises:
        openai.OpenAIError: If translation fails

    Example:
        >>> translate_urdu_to_english("PID کیا ہے؟")
        "What is a PID controller?"
    """
    try:
        # Extract technical terms to preserve
        tech_terms = extract_technical_terms(urdu_query)
        tech_terms_str = ", ".join(tech_terms) if tech_terms else "none"

        client = get_openai_client()
        response = client.chat.completions.create(
            model="gpt-4o-mini",  # Fast and cost-effective
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You are a professional translator specializing in technical content. "
                        "Translate the following Urdu text to English. "
                        "Preserve all technical terms exactly as they appear. "
                        f"Technical terms to preserve: {tech_terms_str}. "
                        "Output ONLY the English translation, nothing else."
                    )
                },
                {
                    "role": "user",
                    "content": urdu_query
                }
            ],
            temperature=0.3,  # Low temperature for consistent translations
            max_tokens=500
        )

        translation = response.choices[0].message.content.strip()
        return translation

    except Exception as e:
        # Log error (in production, use proper logging)
        print(f"Translation error (Urdu→English): {str(e)}")
        # Fallback: return original query
        return urdu_query


def translate_english_to_urdu(
    english_response: str,
    preserve_terms: Optional[List[str]] = None
) -> str:
    """
    Translate English response to Urdu while preserving technical terms.

    Technical terms are kept in English or shown with Urdu translation
    in parentheses: "پی آئی ڈی کنٹرولر (PID Controller)"

    Args:
        english_response: Response text in English
        preserve_terms: List of technical terms to preserve (auto-detected if None)

    Returns:
        Translated response in Urdu with preserved technical terms

    Raises:
        openai.OpenAIError: If translation fails

    Example:
        >>> translate_english_to_urdu(
        ...     "A PID controller is a control loop mechanism",
        ...     preserve_terms=["PID controller"]
        ... )
        "PID Controller ایک کنٹرول لوپ میکانزم ہے"
    """
    try:
        # Auto-detect technical terms if not provided
        if preserve_terms is None:
            preserve_terms = extract_technical_terms(english_response)

        tech_terms_str = ", ".join(preserve_terms) if preserve_terms else "none"

        client = get_openai_client()
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You are a professional translator specializing in robotics and AI technical content. "
                        "Translate the following English text to Urdu. "
                        "IMPORTANT: Keep ALL technical terms in English exactly as they appear. "
                        "Do NOT translate technical terms to Urdu. "
                        f"Technical terms to preserve in English: {tech_terms_str}. "
                        "Use clear, natural Urdu for the rest of the text. "
                        "Output ONLY the Urdu translation, nothing else."
                    )
                },
                {
                    "role": "user",
                    "content": english_response
                }
            ],
            temperature=0.3,
            max_tokens=1000
        )

        translation = response.choices[0].message.content.strip()
        return translation

    except Exception as e:
        # Log error
        print(f"Translation error (English→Urdu): {str(e)}")
        # Fallback: return original English response
        return english_response


def translate_bidirectional(
    text: str,
    source_lang: str,
    target_lang: str,
    preserve_terms: Optional[List[str]] = None
) -> str:
    """
    Translate text bidirectionally (English ↔ Urdu).

    Convenience function that routes to the appropriate translation function.

    Args:
        text: Input text to translate
        source_lang: Source language code ('en' or 'ur')
        target_lang: Target language code ('en' or 'ur')
        preserve_terms: Technical terms to preserve (optional)

    Returns:
        Translated text

    Raises:
        ValueError: If language codes are invalid
        openai.OpenAIError: If translation fails

    Example:
        >>> translate_bidirectional("Hello world", "en", "ur")
        "ہیلو ورلڈ"
    """
    if source_lang == target_lang:
        return text  # No translation needed

    if source_lang == "ur" and target_lang == "en":
        return translate_urdu_to_english(text)
    elif source_lang == "en" and target_lang == "ur":
        return translate_english_to_urdu(text, preserve_terms)
    else:
        raise ValueError(
            f"Unsupported language pair: {source_lang} → {target_lang}. "
            "Only 'en' and 'ur' are supported."
        )


# For backward compatibility and testing
if __name__ == "__main__":
    # Test language detection
    print("Language Detection Tests:")
    print(detect_language("What is a PID controller?"))
    print(detect_language("PID کیا ہے؟"))
    print(detect_language("ROS 2 میں navigation کیسے کام کرتا ہے؟"))

    # Test technical term extraction
    print("\nTechnical Term Extraction:")
    print(extract_technical_terms("ROS 2, Gazebo, and PID Controller"))

    # Note: Actual translation tests require OpenAI API key
    print("\nTranslation tests require OPENAI_API_KEY environment variable")
