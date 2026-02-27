#!/usr/bin/env python3
"""
Training Data Visualization Server
====================================

Flask server that loads binary training data files and serves them
to a WebGL frontend for 3D animated replay of training scenarios.

Usage:
    uv run python vis/training/server.py --data-dir data/simulated
"""

import argparse
import logging
import os
import struct
import sys
from pathlib import Path
from typing import Optional

from flask import Flask, Response, jsonify, send_from_directory

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__, static_folder='static')

# Global data directory
data_dir: Optional[Path] = None

# Binary format constants (must match data_generator.py)
BIN_MAGIC = b'APFD'
BIN_HEADER_SIZE = 16  # 4 magic + 4 version + 4 feature_dim + 4 reserved
BIN_COLS = 23          # 22 features + 1 label


def sanitize_path(base_dir: Path, name: str) -> Optional[Path]:
    """Sanitize a user-provided filename to prevent path traversal."""
    if not name or not name.strip():
        return None
    base_path = os.path.abspath(str(base_dir))
    full_path = os.path.normpath(os.path.join(base_path, name))
    if not full_path.startswith(base_path + os.sep) and full_path != base_path:
        return None
    return Path(full_path)


@app.route('/')
def index():
    """Serve the main HTML page."""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/static/<path:filename>')
def static_files(filename):
    """Serve static files."""
    return send_from_directory(app.static_folder, filename)


@app.route('/api/files')
def list_files():
    """List .bin files in data directory with metadata."""
    if data_dir is None or not data_dir.exists():
        return jsonify({'files': [], 'error': 'Data directory not configured'})

    files = []
    for f in sorted(data_dir.glob('*.bin')):
        try:
            size = f.stat().st_size
            if size < BIN_HEADER_SIZE:
                continue

            with open(f, 'rb') as fh:
                magic = fh.read(4)
                if magic != BIN_MAGIC:
                    continue
                version, feature_dim, _ = struct.unpack('<III', fh.read(12))

            cols = feature_dim + 1  # features + label
            data_bytes = size - BIN_HEADER_SIZE
            frame_count = data_bytes // (cols * 4)
            duration = frame_count / 2.0  # 2 Hz sample rate

            files.append({
                'name': f.name,
                'frames': frame_count,
                'duration': round(duration, 1),
                'size': size,
                'feature_dim': feature_dim,
            })
        except Exception:
            logger.debug(f"Skipping invalid file: {f.name}")
            continue

    return jsonify({'files': files})


@app.route('/api/data/<filename>')
def get_data(filename):
    """Return raw binary frame data (header stripped)."""
    if data_dir is None:
        return jsonify({'error': 'Data directory not configured'}), 500

    file_path = sanitize_path(data_dir, filename)
    if file_path is None:
        return jsonify({'error': 'Invalid filename'}), 400

    if not file_path.exists() or not file_path.suffix == '.bin':
        return jsonify({'error': 'File not found'}), 404

    try:
        with open(file_path, 'rb') as f:
            magic = f.read(4)
            if magic != BIN_MAGIC:
                return jsonify({'error': 'Invalid file format'}), 400

            version, feature_dim, _ = struct.unpack('<III', f.read(12))
            frame_data = f.read()

        cols = feature_dim + 1
        frame_count = len(frame_data) // (cols * 4)

        resp = Response(frame_data, mimetype='application/octet-stream')
        resp.headers['X-Frame-Count'] = str(frame_count)
        resp.headers['X-Feature-Dim'] = str(feature_dim)
        resp.headers['X-Cols'] = str(cols)
        resp.headers['Access-Control-Expose-Headers'] = 'X-Frame-Count, X-Feature-Dim, X-Cols'
        return resp

    except Exception:
        logger.exception("Failed to read data file")
        return jsonify({'error': 'Failed to read file'}), 500


def main():
    global data_dir

    parser = argparse.ArgumentParser(description='Training Data Visualization Server')
    parser.add_argument('--data-dir', '-d', default='data/simulated',
                        help='Directory containing .bin training data (default: data/simulated)')
    parser.add_argument('--port', '-p', type=int, default=8081,
                        help='Port to run server on (default: 8081)')
    parser.add_argument('--host', default='127.0.0.1',
                        help='Host to bind to (default: 127.0.0.1)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode')

    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    if not data_dir.exists():
        logger.error(f"Data directory not found: {data_dir}")
        sys.exit(1)

    bin_count = len(list(data_dir.glob('*.bin')))
    logger.info(f"Data directory: {data_dir} ({bin_count} .bin files)")
    logger.info(f"Starting server at http://{args.host}:{args.port}")

    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == '__main__':
    main()
