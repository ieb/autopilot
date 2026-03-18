"""
CAN Logger Web UI
=================

Flask app providing REST API, SSE streaming, and static dashboard
for monitoring and configuring the CAN frame logger.
"""

import json
import logging
import os
import tempfile
import time
from pathlib import Path
from typing import Optional

from flask import Flask, Response, jsonify, request, send_from_directory

from src.n2k.can_logger import CANLogger, CANLoggerConfig, PGN_NAMES

logger = logging.getLogger(__name__)


def sanitize_path(base_dir: Path, name: str) -> Optional[Path]:
    """Prevent directory traversal attacks by enforcing containment within base_dir."""
    if not name or not name.strip():
        return None

    base_path = Path(base_dir).resolve()
    try:
        full_path = (base_path / name).resolve()
    except (OSError, RuntimeError):
        # Resolution failed (e.g., invalid path); treat as unsafe.
        return None

    try:
        # Ensure full_path is inside base_path (or equal to it)
        full_path.relative_to(base_path)
    except ValueError:
        return None

    return full_path


def create_app(can_logger: CANLogger, config_path: Optional[str] = None) -> Flask:
    """Create Flask app wired to a CANLogger instance."""
    static_dir = Path(__file__).parent / "can_logger_static"
    app = Flask(__name__, static_folder=str(static_dir))

    @app.route("/")
    def index():
        return send_from_directory(str(static_dir), "index.html")

    @app.route("/api/status")
    def api_status():
        return jsonify(can_logger.stats)

    @app.route("/api/config", methods=["GET"])
    def api_get_config():
        return jsonify(can_logger.config.to_dict())

    @app.route("/api/config", methods=["POST"])
    def api_post_config():
        if not config_path:
            return jsonify({"error": "No config file configured"}), 400

        try:
            updates = request.get_json(force=True)
        except Exception:
            return jsonify({"error": "Invalid JSON"}), 400

        # Only allow safe config fields
        safe_fields = {
            "include", "exclude", "rate_limits",
            "max_file_size_mb", "max_disk_mb", "gps_time_sync",
        }
        filtered = {k: v for k, v in updates.items() if k in safe_fields}
        if not filtered:
            return jsonify({"error": "No valid fields provided"}), 400

        # Merge with current config
        try:
            current = can_logger.config.to_dict()
            current.update(filtered)

            # Atomic write: temp file + rename
            config_dir = os.path.dirname(os.path.abspath(config_path))
            os.makedirs(config_dir, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(dir=config_dir, suffix=".json.tmp")
            try:
                with os.fdopen(fd, "w") as f:
                    json.dump(current, f, indent=2)
                os.replace(tmp_path, config_path)
            except Exception:
                os.unlink(tmp_path)
                raise

            logger.info("Config updated via web UI")
            return jsonify({"status": "ok"})
        except Exception as e:
            logger.error("Config update failed: %s", e)
            return jsonify({"error": "Failed to update config"}), 500

    @app.route("/api/files")
    def api_files():
        log_dir = Path(can_logger.config.log_dir)
        if not log_dir.exists():
            return jsonify([])

        files = []
        for p in sorted(log_dir.glob("can_*.bin"), key=lambda x: x.stat().st_mtime, reverse=True):
            stat = p.stat()
            from src.n2k.can_logger import HEADER_SIZE, FRAME_SIZE
            frames = max(0, (stat.st_size - HEADER_SIZE)) // FRAME_SIZE if stat.st_size >= HEADER_SIZE else 0
            files.append({
                "name": p.name,
                "size": stat.st_size,
                "frames": frames,
                "modified": stat.st_mtime,
            })
        return jsonify(files)

    @app.route("/api/files/<path:name>")
    def api_download_file(name):
        log_dir = Path(can_logger.config.log_dir)
        safe_path = sanitize_path(log_dir, name)

        if safe_path is None or not safe_path.exists():
            return jsonify({"error": "File not found"}), 404

        if not safe_path.name.startswith("can_") or not safe_path.name.endswith(".bin"):
            return jsonify({"error": "Invalid file"}), 400

        return send_from_directory(str(log_dir), safe_path.name, as_attachment=True)

    @app.route("/api/pgn_names")
    def api_pgn_names():
        return jsonify({str(k): v for k, v in PGN_NAMES.items()})

    @app.route("/stream")
    def stream():
        """SSE endpoint: recent frames + stats at 2 Hz."""
        def generate():
            while True:
                frames = can_logger.get_recent_frames(20)
                stats = can_logger.stats
                data = {
                    "frames": frames,
                    "stats": stats,
                }
                yield f"data: {json.dumps(data)}\n\n"
                time.sleep(0.5)

        return Response(
            generate(),
            mimetype="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    return app
