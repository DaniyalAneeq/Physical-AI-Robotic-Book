"""Translation cache service for chatbot query translations."""

import hashlib
from datetime import datetime
from typing import Optional

from sqlalchemy import text
from sqlalchemy.orm import Session


def compute_query_hash(query: str, source_lang: str, target_lang: str) -> str:
    """
    Compute SHA-256 hash for translation cache key.

    Normalizes query to improve cache hit rate:
    - Lowercase
    - Trim whitespace
    - Remove punctuation except spaces

    Args:
        query: Original query text
        source_lang: Source language code ('en' or 'ur')
        target_lang: Target language code ('en' or 'ur')

    Returns:
        64-character SHA-256 hash

    Examples:
        >>> compute_query_hash("PID کیا ہے؟", "ur", "en")
        '...'  # 64-char hash
        >>> compute_query_hash("PID کیا ہے", "ur", "en")  # Same (punctuation removed)
        '...'  # Same hash
    """
    # Normalize query
    normalized = query.lower().strip()

    # Remove punctuation except spaces and non-ASCII characters
    # Keep Urdu characters for proper matching
    normalized = "".join(
        c for c in normalized
        if c.isalnum() or c.isspace() or ord(c) > 127  # Keep non-ASCII (Urdu)
    )

    # Create composite key: normalized_query|source|target
    composite_key = f"{normalized}|{source_lang}|{target_lang}"

    # Compute SHA-256 hash
    return hashlib.sha256(composite_key.encode("utf-8")).hexdigest()


def get_cached_translation(query_hash: str, db: Session) -> Optional[str]:
    """
    Retrieve cached translation if exists.

    Updates last_accessed timestamp and increments access_count.

    Args:
        query_hash: SHA-256 hash from compute_query_hash()
        db: Database session

    Returns:
        Translated response text if cached, None otherwise
    """
    # Query cache table
    result = db.execute(
        text("""
            SELECT translated_response
            FROM translation_cache
            WHERE query_hash = :query_hash
            LIMIT 1
        """),
        {"query_hash": query_hash}
    ).fetchone()

    if result:
        # Cache hit - update stats
        db.execute(
            text("""
                UPDATE translation_cache
                SET last_accessed = :now,
                    access_count = access_count + 1
                WHERE query_hash = :query_hash
            """),
            {
                "query_hash": query_hash,
                "now": datetime.utcnow()
            }
        )
        db.commit()

        return result[0]  # Return translated_response

    # Cache miss
    return None


def cache_translation(
    query_hash: str,
    query: str,
    response: str,
    source_lang: str,
    target_lang: str,
    db: Session
) -> None:
    """
    Store translation in cache.

    Args:
        query_hash: SHA-256 hash from compute_query_hash()
        query: Original query text
        response: Translated response text
        source_lang: Source language code
        target_lang: Target language code
        db: Database session
    """
    # Insert new cache entry (ignore if already exists due to race condition)
    db.execute(
        text("""
            INSERT INTO translation_cache (
                query_hash,
                source_language,
                target_language,
                original_query,
                translated_response,
                created_at,
                last_accessed,
                access_count
            ) VALUES (
                :query_hash,
                :source_lang,
                :target_lang,
                :query,
                :response,
                :now,
                :now,
                1
            )
            ON CONFLICT (query_hash) DO NOTHING
        """),
        {
            "query_hash": query_hash,
            "source_lang": source_lang,
            "target_lang": target_lang,
            "query": query,
            "response": response,
            "now": datetime.utcnow()
        }
    )
    db.commit()


def get_cache_stats(db: Session) -> dict:
    """
    Get translation cache statistics.

    Returns:
        Dictionary with cache metrics:
        - total_entries: Total cached translations
        - avg_access_count: Average hits per cached query
        - cache_hit_rate: Estimated hit rate (avg_access_count / total)
    """
    result = db.execute(
        text("""
            SELECT
                COUNT(*) as total_entries,
                AVG(access_count) as avg_access_count,
                SUM(access_count) as total_hits
            FROM translation_cache
        """)
    ).fetchone()

    if result and result[0] > 0:
        total_entries = result[0]
        avg_access_count = float(result[1]) if result[1] else 0
        total_hits = result[2] if result[2] else 0

        # Estimated hit rate: (total_hits - total_entries) / total_hits
        # Assumes first access is miss, subsequent accesses are hits
        cache_hit_rate = 0.0
        if total_hits > 0:
            cache_hit_rate = (total_hits - total_entries) / total_hits

        return {
            "total_entries": total_entries,
            "avg_access_count": avg_access_count,
            "total_hits": total_hits,
            "cache_hit_rate": cache_hit_rate
        }

    return {
        "total_entries": 0,
        "avg_access_count": 0.0,
        "total_hits": 0,
        "cache_hit_rate": 0.0
    }
