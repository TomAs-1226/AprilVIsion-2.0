#!/usr/bin/env python3
"""
AprilVision 2.0 - Reverse Proxy with Custom Branding

This script runs as a reverse proxy in front of PhotonVision's web server.
It passes all requests through to PhotonVision but rewrites HTML responses
to replace PhotonVision branding with custom AprilVision branding.

Architecture:
  - PhotonVision runs on port 5800 (internal, detection + native UI)
  - This proxy runs on port 5801 (the port teams access)
  - All requests are forwarded to PhotonVision
  - HTML/JS responses get branding replaced
  - Camera streams are proxied transparently

Usage:
  python3 aprilvision-bridge.py [--pv-port 5800] [--port 5801]
"""

import http.server
import http.client
import socketserver
import threading
import sys
import time
import gzip
import io
import re

# Configuration
PROXY_PORT = 5801
PHOTONVISION_HOST = "127.0.0.1"
PHOTONVISION_PORT = 5800

# Branding replacements applied to HTML and JS content
BRANDING_REPLACEMENTS = [
    # Main title replacements
    (b"PhotonVision", b"AprilVision 2.0"),
    (b"photonvision", b"aprilvision"),
    (b"Photon Vision", b"AprilVision 2.0"),
    # Keep functional references intact by being selective
    # We replace display text but not internal API paths
]

# CSS injection for custom styling
CUSTOM_CSS = b"""
<style id="aprilvision-custom">
  /* AprilVision 2.0 Custom Branding */
  .photonvision-header, [class*="header"] {
    position: relative;
  }
  /* Subtle branding indicator in bottom-right */
  body::after {
    content: "Custom Vision System | AprilVision 2.0";
    position: fixed;
    bottom: 4px;
    right: 8px;
    font-size: 10px;
    color: rgba(255,255,255,0.3);
    z-index: 99999;
    pointer-events: none;
  }
</style>
"""

# JS injection for runtime branding
CUSTOM_JS = b"""
<script id="aprilvision-rebrand">
(function() {
  // Replace PhotonVision text in the DOM after page loads
  function rebrand() {
    // Replace document title
    if (document.title.includes('PhotonVision') || document.title.includes('photonvision')) {
      document.title = document.title
        .replace(/PhotonVision/gi, 'AprilVision 2.0');
    } else if (!document.title.includes('AprilVision')) {
      document.title = 'AprilVision 2.0 Dashboard';
    }

    // Walk visible text nodes and replace branding
    var walker = document.createTreeWalker(
      document.body, NodeFilter.SHOW_TEXT, null, false
    );
    var node;
    while (node = walker.nextNode()) {
      if (node.nodeValue && /photonvision/i.test(node.nodeValue)) {
        node.nodeValue = node.nodeValue
          .replace(/PhotonVision/g, 'AprilVision 2.0')
          .replace(/Photon Vision/g, 'AprilVision 2.0')
          .replace(/photonvision/g, 'AprilVision 2.0');
      }
    }

    // Replace in alt/title attributes
    document.querySelectorAll('[alt*="PhotonVision"], [alt*="photonvision"], [title*="PhotonVision"], [title*="photonvision"]').forEach(function(el) {
      if (el.alt) el.alt = el.alt.replace(/PhotonVision|photonvision|Photon Vision/gi, 'AprilVision 2.0');
      if (el.title) el.title = el.title.replace(/PhotonVision|photonvision|Photon Vision/gi, 'AprilVision 2.0');
    });
  }

  // Run on load and periodically (for SPA navigation)
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', rebrand);
  } else {
    rebrand();
  }
  // Re-run periodically for Vue/React SPA updates
  setInterval(rebrand, 2000);
})();
</script>
"""


def apply_branding(content, content_type):
    """Apply custom branding to response content."""
    if not content_type:
        return content

    ct = content_type.lower()

    # Only modify HTML and JS content
    if "text/html" not in ct and "javascript" not in ct and "text/css" not in ct:
        return content

    # Apply text replacements
    modified = content
    for old, new in BRANDING_REPLACEMENTS:
        modified = modified.replace(old, new)

    # Inject custom CSS and JS into HTML pages
    if b"text/html" in content_type.encode() or b"</head>" in modified or b"</body>" in modified:
        # Inject CSS before </head>
        if b"</head>" in modified:
            modified = modified.replace(b"</head>", CUSTOM_CSS + b"</head>", 1)

        # Inject JS before </body>
        if b"</body>" in modified:
            modified = modified.replace(b"</body>", CUSTOM_JS + b"</body>", 1)

    return modified


class ReverseProxyHandler(http.server.BaseHTTPRequestHandler):
    """Reverse proxy that forwards requests to PhotonVision and rebrands responses."""

    # Suppress default logging for clean output
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        self._proxy_request("GET")

    def do_POST(self):
        self._proxy_request("POST")

    def do_PUT(self):
        self._proxy_request("PUT")

    def do_DELETE(self):
        self._proxy_request("DELETE")

    def do_OPTIONS(self):
        self._proxy_request("OPTIONS")

    def _proxy_request(self, method):
        """Forward a request to PhotonVision and return the rebranded response."""
        try:
            # Read request body if present
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else None

            # Connect to PhotonVision
            conn = http.client.HTTPConnection(
                PHOTONVISION_HOST, PHOTONVISION_PORT, timeout=30
            )

            # Forward headers (strip host, add correct one)
            headers = {}
            for key, value in self.headers.items():
                if key.lower() not in ("host", "accept-encoding"):
                    headers[key] = value
            headers["Host"] = f"{PHOTONVISION_HOST}:{PHOTONVISION_PORT}"
            # Request uncompressed content so we can modify it
            headers["Accept-Encoding"] = "identity"

            # Send request
            conn.request(method, self.path, body=body, headers=headers)
            response = conn.getresponse()

            # Read response
            resp_body = response.read()
            content_type = response.getheader("Content-Type", "")

            # Check if content is gzipped despite our request
            content_encoding = response.getheader("Content-Encoding", "")
            if "gzip" in content_encoding.lower() and resp_body:
                try:
                    resp_body = gzip.decompress(resp_body)
                    content_encoding = ""
                except Exception:
                    pass

            # Apply branding to text content
            is_stream = "multipart" in content_type.lower() or "octet-stream" in content_type.lower()
            if not is_stream and resp_body:
                resp_body = apply_branding(resp_body, content_type)

            # Send response
            self.send_response(response.status)

            # Forward response headers
            for key, value in response.getheaders():
                lower_key = key.lower()
                # Skip headers we'll set ourselves
                if lower_key in ("content-length", "transfer-encoding", "content-encoding", "connection"):
                    continue
                self.send_header(key, value)

            # Set correct content length after modification
            if resp_body is not None:
                self.send_header("Content-Length", str(len(resp_body)))

            self.end_headers()

            # Send body
            if resp_body:
                self.wfile.write(resp_body)

            conn.close()

        except ConnectionRefusedError:
            self._send_error_page(
                503,
                "PhotonVision Not Running",
                "The PhotonVision detection engine is not responding. "
                "Check that the photonvision service is running: "
                "sudo systemctl status photonvision"
            )
        except Exception as e:
            self._send_error_page(
                502,
                "Proxy Error",
                f"Failed to connect to PhotonVision backend: {e}"
            )

    def _send_error_page(self, code, title, message):
        """Send a branded error page."""
        body = f"""<!DOCTYPE html>
<html>
<head>
    <title>AprilVision 2.0 - {title}</title>
    <style>
        body {{
            background: #1a1a2e;
            color: #e0e0e0;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
        }}
        .error-box {{
            background: #16213e;
            border: 1px solid #0f3460;
            border-radius: 12px;
            padding: 40px;
            max-width: 500px;
            text-align: center;
        }}
        h1 {{ color: #e94560; margin-bottom: 10px; }}
        h2 {{ color: #00d9ff; margin-bottom: 20px; font-size: 1.1em; }}
        p {{ color: #a0a0a0; line-height: 1.6; }}
        .brand {{ color: #00ff88; font-size: 0.9em; margin-top: 20px; }}
        code {{
            background: #0a0a1a;
            padding: 2px 8px;
            border-radius: 4px;
            font-size: 0.9em;
        }}
    </style>
    <meta http-equiv="refresh" content="5">
</head>
<body>
    <div class="error-box">
        <h1>{code}</h1>
        <h2>{title}</h2>
        <p>{message}</p>
        <p style="color:#666;font-size:0.85em;">This page will auto-refresh in 5 seconds.</p>
        <div class="brand">AprilVision 2.0 - Custom Vision System</div>
    </div>
</body>
</html>""".encode("utf-8")

        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class StreamingProxyHandler(ReverseProxyHandler):
    """Extended handler with streaming support for MJPEG camera feeds."""

    def _proxy_request(self, method):
        """Override to handle streaming responses (MJPEG, SSE)."""
        # Check if this is a stream request
        is_stream_path = (
            "/api/stream" in self.path or
            ".mjpeg" in self.path or
            "stream" in self.path.lower()
        )

        if is_stream_path and method == "GET":
            self._proxy_stream()
        else:
            super()._proxy_request(method)

    def _proxy_stream(self):
        """Proxy a streaming response (MJPEG) without buffering."""
        try:
            conn = http.client.HTTPConnection(
                PHOTONVISION_HOST, PHOTONVISION_PORT, timeout=60
            )
            headers = {}
            for key, value in self.headers.items():
                if key.lower() not in ("host",):
                    headers[key] = value
            headers["Host"] = f"{PHOTONVISION_HOST}:{PHOTONVISION_PORT}"

            conn.request("GET", self.path, headers=headers)
            response = conn.getresponse()

            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() != "transfer-encoding":
                    self.send_header(key, value)
            self.end_headers()

            # Stream data through
            try:
                while True:
                    chunk = response.read(8192)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

            conn.close()

        except Exception:
            self._send_error_page(502, "Stream Error", "Camera stream unavailable")


class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Threaded HTTP server for handling concurrent requests."""
    allow_reuse_address = True
    daemon_threads = True


def main():
    print("=" * 60)
    print("  AprilVision 2.0 - Custom Vision System")
    print("  Reverse proxy with custom branding")
    print("=" * 60)
    print(f"  Proxy port:     {PROXY_PORT}")
    print(f"  PhotonVision:   http://{PHOTONVISION_HOST}:{PHOTONVISION_PORT}")
    print(f"  Dashboard:      http://0.0.0.0:{PROXY_PORT}")
    print()
    print("  All PhotonVision features available through this proxy.")
    print("  PhotonVision branding is replaced with AprilVision 2.0.")
    print("=" * 60)
    print()

    server = ThreadedHTTPServer(("0.0.0.0", PROXY_PORT), StreamingProxyHandler)
    print(f"[AprilVision] Proxy running on port {PROXY_PORT}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[AprilVision] Shutting down...")
        server.shutdown()


if __name__ == "__main__":
    # Parse simple args
    import argparse
    parser = argparse.ArgumentParser(description="AprilVision 2.0 Reverse Proxy")
    parser.add_argument("--port", type=int, default=PROXY_PORT, help="Proxy port")
    parser.add_argument("--pv-port", type=int, default=PHOTONVISION_PORT, help="PhotonVision port")
    args = parser.parse_args()
    PROXY_PORT = args.port
    PHOTONVISION_PORT = args.pv_port
    main()
