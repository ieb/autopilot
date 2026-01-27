"""
Tests for the GRIB visualization server.
"""

import os
import tempfile
from pathlib import Path

import pytest

# Import the sanitize_path function
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from vis.gribs.server import sanitize_path


class TestSanitizePath:
    """Test the sanitize_path function for path traversal prevention."""
    
    @pytest.fixture
    def base_dir(self, tmp_path):
        """Create a temporary base directory with some files."""
        # Create base directory structure
        (tmp_path / "file1.csv").touch()
        (tmp_path / "file2.csv").touch()
        (tmp_path / "subdir").mkdir()
        (tmp_path / "subdir" / "nested.csv").touch()
        return tmp_path
    
    def test_valid_filename(self, base_dir):
        """Test that valid filenames are accepted."""
        result = sanitize_path(base_dir, "file1.csv")
        assert result is not None
        assert result == base_dir / "file1.csv"
    
    def test_valid_filename_not_exists(self, base_dir):
        """Test that valid but non-existent filenames return a path."""
        result = sanitize_path(base_dir, "nonexistent.csv")
        assert result is not None
        assert result == base_dir / "nonexistent.csv"
    
    def test_empty_name_rejected(self, base_dir):
        """Test that empty names are rejected."""
        assert sanitize_path(base_dir, "") is None
        assert sanitize_path(base_dir, "   ") is None
    
    def test_path_traversal_dotdot_rejected(self, base_dir):
        """Test that .. path traversal is rejected."""
        assert sanitize_path(base_dir, "../etc/passwd") is None
        assert sanitize_path(base_dir, "..") is None
        assert sanitize_path(base_dir, "foo/../../../etc/passwd") is None
    
    def test_path_traversal_absolute_rejected(self, base_dir):
        """Test that absolute paths are rejected."""
        assert sanitize_path(base_dir, "/etc/passwd") is None
        assert sanitize_path(base_dir, "/tmp/file.csv") is None
    
    def test_path_with_slashes_rejected(self, base_dir):
        """Test that paths with forward slashes are handled."""
        # This should be rejected as it tries to escape
        result = sanitize_path(base_dir, "subdir/../../../etc/passwd")
        assert result is None
    
    def test_path_with_backslashes_rejected(self, base_dir):
        """Test that paths with backslashes are handled."""
        result = sanitize_path(base_dir, "..\\..\\etc\\passwd")
        # On Unix, backslashes are valid filename chars but normpath handles them
        # The result should either be None or stay within base_dir
        if result is not None:
            assert str(result).startswith(str(base_dir))
    
    def test_subdirectory_access(self, base_dir):
        """Test that subdirectory access is allowed."""
        result = sanitize_path(base_dir, "subdir")
        assert result is not None
        assert result == base_dir / "subdir"
    
    def test_sibling_directory_rejected(self, base_dir):
        """Test that sibling directories cannot be accessed via traversal."""
        # Create a sibling directory
        sibling = base_dir.parent / "sibling"
        sibling.mkdir(exist_ok=True)
        (sibling / "secret.txt").touch()
        
        # Try to access it via path traversal
        result = sanitize_path(base_dir, "../sibling/secret.txt")
        assert result is None
    
    def test_prefix_attack_rejected(self, base_dir):
        """Test that prefix attacks are rejected (e.g., base_dir_evil)."""
        # Create a sibling with similar name prefix
        evil_dir = Path(str(base_dir) + "_evil")
        evil_dir.mkdir(exist_ok=True)
        (evil_dir / "malicious.txt").touch()
        
        # This should not allow access to the evil directory
        # by exploiting startswith without separator check
        result = sanitize_path(base_dir, "../" + base_dir.name + "_evil/malicious.txt")
        assert result is None
        
        # Cleanup
        (evil_dir / "malicious.txt").unlink()
        evil_dir.rmdir()
    
    def test_dot_current_dir(self, base_dir):
        """Test that single dot is handled correctly."""
        result = sanitize_path(base_dir, "./file1.csv")
        # normpath will resolve this to just "file1.csv" effectively
        if result is not None:
            assert str(result).startswith(str(base_dir))
    
    def test_encoded_traversal_not_decoded(self, base_dir):
        """Test that URL-encoded traversal attempts are treated as literal filenames."""
        # %2F is URL-encoded /, %2E is .
        # These should be treated as literal characters in the filename
        result = sanitize_path(base_dir, "..%2F..%2Fetc%2Fpasswd")
        # This is a weird filename but stays in base_dir
        if result is not None:
            assert str(result).startswith(str(base_dir))
    
    def test_none_name_rejected(self, base_dir):
        """Test that None name is rejected."""
        assert sanitize_path(base_dir, None) is None
    
    def test_unicode_filename(self, base_dir):
        """Test that unicode filenames are handled."""
        result = sanitize_path(base_dir, "文件.csv")
        assert result is not None
        assert result == base_dir / "文件.csv"
    
    def test_whitespace_in_filename(self, base_dir):
        """Test that filenames with whitespace are allowed."""
        result = sanitize_path(base_dir, "my file.csv")
        assert result is not None
        assert result == base_dir / "my file.csv"
