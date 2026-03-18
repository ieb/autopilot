"""Shared path sanitization for vis servers."""

import logging
import os
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


def sanitize_path(base_dir: Path, name: str) -> Optional[Path]:
    """
    Sanitize a user-provided filename to prevent path traversal attacks.

    Args:
        base_dir: The allowed base directory
        name: User-provided filename or directory name

    Returns:
        Sanitized Path if valid, None if the path is unsafe
    """
    # Reject empty names
    if not name or not name.strip():
        return None

    # Get absolute base path
    base_path = os.path.abspath(str(base_dir))

    # Normalize the full path to resolve any .. or . components
    full_path = os.path.normpath(os.path.join(base_path, name))

    # Verify the normalized path is still under the base directory
    # Must start with base_path + separator to prevent prefix attacks
    # (e.g., /base/dir vs /base/dir_other)
    if not full_path.startswith(base_path + os.sep) and full_path != base_path:
        logger.warning("Path traversal attempt blocked: %s", name)
        return None

    return Path(full_path)
