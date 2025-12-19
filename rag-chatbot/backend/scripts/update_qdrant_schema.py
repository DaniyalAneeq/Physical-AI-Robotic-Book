# #!/usr/bin/env python3
# """
# Update Qdrant schema to add i18n support.

# This script adds dual-language fields to existing Qdrant points:
# - content_ur: Urdu content text (nullable)
# - title_ur: Urdu title text (nullable)
# - translation_status: Enum ('complete', 'pending', 'partial', 'unavailable')
# - technical_terms: Array of terms to preserve in English

# This is a NON-DESTRUCTIVE operation - all existing English content is preserved.
# """

# import os
# import sys
# from pathlib import Path

# # Add parent directory to path for imports
# sys.path.insert(0, str(Path(__file__).parent.parent))

# from dotenv import load_dotenv
# from qdrant_client import QdrantClient
# from qdrant_client.models import PointStruct

# # Load environment variables
# load_dotenv()

# QDRANT_URL = os.getenv("QDRANT_URL")
# QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
# COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content")


# def update_qdrant_schema(dry_run: bool = True) -> None:
#     """
#     Update Qdrant collection schema to support bilingual content.

#     Args:
#         dry_run: If True, only print changes without applying them
#     """
#     if not QDRANT_URL or not QDRANT_API_KEY:
#         raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment")

#     print(f"{'[DRY RUN] ' if dry_run else ''}Connecting to Qdrant at {QDRANT_URL}")
#     client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

#     # Verify collection exists
#     try:
#         collection_info = client.get_collection(collection_name=COLLECTION_NAME)
#         print(f"✓ Collection '{COLLECTION_NAME}' found")
#         print(f"  Total points: {collection_info.points_count}")
#     except Exception as e:
#         print(f"✗ Error: Collection '{COLLECTION_NAME}' not found: {e}")
#         return

#     # Scroll through all points and update payloads
#     offset = None
#     batch_size = 100
#     total_updated = 0
#     points_to_update = []

#     print(f"\n{'[DRY RUN] ' if dry_run else ''}Updating point payloads...")

#     while True:
#         # Scroll through points
#         points, next_offset = client.scroll(
#             collection_name=COLLECTION_NAME,
#             limit=batch_size,
#             offset=offset,
#             with_payload=True,
#             with_vectors=False,  # Don't need vectors, saves bandwidth
#         )

#         if not points:
#             break

#         for point in points:
#             # Check if already updated (has content_ur field)
#             if "content_ur" in point.payload:
#                 continue  # Skip already updated points

#             # Add new i18n fields to payload
#             updated_payload = point.payload.copy()
#             updated_payload["content_ur"] = None
#             updated_payload["title_ur"] = None
#             updated_payload["translation_status"] = "pending"
#             updated_payload["technical_terms"] = []

#             # Create updated point
#             updated_point = PointStruct(
#                 id=point.id,
#                 vector=point.vector,  # Preserve original vector
#                 payload=updated_payload,
#             )
#             points_to_update.append(updated_point)

#         # Batch upsert
#         if points_to_update:
#             if not dry_run:
#                 client.upsert(
#                     collection_name=COLLECTION_NAME,
#                     points=points_to_update,
#                     wait=True,
#                 )
#             total_updated += len(points_to_update)
#             print(f"  {'Would update' if dry_run else 'Updated'} {len(points_to_update)} points")
#             points_to_update = []

#         # Check for more points
#         if next_offset is None:
#             break
#         offset = next_offset

#     print(f"\n{'[DRY RUN] ' if dry_run else ''}Summary:")
#     print(f"  Total points {'that would be updated' if dry_run else 'updated'}: {total_updated}")

#     if dry_run:
#         print("\nTo apply changes, run with --apply flag")
#     else:
#         print("\n✓ Qdrant schema update complete!")
#         print("  All English content preserved.")
#         print("  Ready for Urdu content ingestion.")


# def verify_schema_update() -> None:
#     """Verify that schema update was successful."""
#     client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

#     # Retrieve a sample point
#     points, _ = client.scroll(
#         collection_name=COLLECTION_NAME,
#         limit=1,
#         with_payload=True,
#         with_vectors=False,
#     )

#     if not points:
#         print("✗ No points found in collection")
#         return

#     sample_point = points[0]
#     required_fields = ["content_ur", "title_ur", "translation_status", "technical_terms"]

#     print("\nVerification:")
#     all_present = True
#     for field in required_fields:
#         present = field in sample_point.payload
#         status = "✓" if present else "✗"
#         print(f"  {status} {field}: {present}")
#         if not present:
#             all_present = False

#     if all_present:
#         print("\n✓ Schema verification passed!")
#     else:
#         print("\n✗ Schema verification failed - some fields missing")


# if __name__ == "__main__":
#     import argparse

#     parser = argparse.ArgumentParser(
#         description="Update Qdrant schema for i18n support"
#     )
#     parser.add_argument(
#         "--apply",
#         action="store_true",
#         help="Apply changes (default is dry-run)",
#     )
#     parser.add_argument(
#         "--verify",
#         action="store_true",
#         help="Verify schema update after applying",
#     )

#     args = parser.parse_args()

#     try:
#         update_qdrant_schema(dry_run=not args.apply)

#         if args.verify and args.apply:
#             print("\n" + "=" * 50)
#             verify_schema_update()

#     except Exception as e:
#         print(f"✗ Error: {e}")
#         sys.exit(1)

#!/usr/bin/env python3
"""
Update Qdrant schema to add i18n support.

This script adds dual-language fields to existing Qdrant points:
- content_ur: Urdu content text (nullable)
- title_ur: Urdu title text (nullable)
- translation_status: Enum ('complete', 'pending', 'partial', 'unavailable')
- technical_terms: Array of terms to preserve in English

This is a NON-DESTRUCTIVE operation - all existing English content is preserved.
"""

import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content")


def update_qdrant_schema(dry_run: bool = True) -> None:
    """
    Update Qdrant collection schema to support bilingual content.

    Args:
        dry_run: If True, only print changes without applying them
    """
    if not QDRANT_URL or not QDRANT_API_KEY:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment")

    print(f"{'[DRY RUN] ' if dry_run else ''}Connecting to Qdrant at {QDRANT_URL}")
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # Verify collection exists
    try:
        collection_info = client.get_collection(collection_name=COLLECTION_NAME)
        print(f"✓ Collection '{COLLECTION_NAME}' found")
        print(f"  Total points: {collection_info.points_count}")
    except Exception as e:
        print(f"✗ Error: Collection '{COLLECTION_NAME}' not found: {e}")
        return

    offset = None
    batch_size = 100
    total_updated = 0

    print(f"\n{'[DRY RUN] ' if dry_run else ''}Updating point payloads...")

    while True:
        points, next_offset = client.scroll(
            collection_name=COLLECTION_NAME,
            limit=batch_size,
            offset=offset,
            with_payload=True,
            with_vectors=False,  # vectors NOT needed
        )

        if not points:
            break

        batch_count = 0

        for point in points:
            # Skip already updated points
            if "content_ur" in point.payload:
                continue

            # Merge new fields with existing payload
            updated_payload = point.payload.copy()
            updated_payload["content_ur"] = None
            updated_payload["title_ur"] = None
            updated_payload["translation_status"] = "pending"
            updated_payload["technical_terms"] = []

            if not dry_run:
                client.overwrite_payload(
                    collection_name=COLLECTION_NAME,
                    payload=updated_payload,
                    points=[point.id],
                )

            batch_count += 1
            total_updated += 1

        print(f"  {'Would update' if dry_run else 'Updated'} {batch_count} points")

        if next_offset is None:
            break
        offset = next_offset

    print(f"\n{'[DRY RUN] ' if dry_run else ''}Summary:")
    print(f"  Total points {'that would be updated' if dry_run else 'updated'}: {total_updated}")

    if dry_run:
        print("\nTo apply changes, run with --apply flag")
    else:
        print("\n✓ Qdrant schema update complete!")
        print("  All English content preserved.")
        print("  Ready for Urdu content ingestion.")


def verify_schema_update() -> None:
    """Verify that schema update was successful."""
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    points, _ = client.scroll(
        collection_name=COLLECTION_NAME,
        limit=1,
        with_payload=True,
        with_vectors=False,
    )

    if not points:
        print("✗ No points found in collection")
        return

    sample_point = points[0]
    required_fields = [
        "content_ur",
        "title_ur",
        "translation_status",
        "technical_terms",
    ]

    print("\nVerification:")
    all_present = True
    for field in required_fields:
        present = field in sample_point.payload
        status = "✓" if present else "✗"
        print(f"  {status} {field}: {present}")
        if not present:
            all_present = False

    if all_present:
        print("\n✓ Schema verification passed!")
    else:
        print("\n✗ Schema verification failed - some fields missing")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Update Qdrant schema for i18n support"
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Apply changes (default is dry-run)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Verify schema update after applying",
    )

    args = parser.parse_args()

    try:
        update_qdrant_schema(dry_run=not args.apply)

        if args.verify and args.apply:
            print("\n" + "=" * 50)
            verify_schema_update()

    except Exception as e:
        print(f"✗ Error: {e}")
        sys.exit(1)
