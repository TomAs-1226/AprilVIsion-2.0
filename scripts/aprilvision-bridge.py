#!/usr/bin/env python3
"""
AprilVision 3.2 - FRC Competition Vision System
Reverse proxy with branding, WebSocket tunneling, and system monitoring

Built by Team 1226.

Architecture:
  - Detection engine runs on port 5800 (internal, not exposed)
  - This proxy runs on port 5801 (team-facing dashboard)
  - All HTTP requests forwarded and rebranded transparently
  - WebSocket connections tunneled for real-time camera data
  - Camera MJPEG streams proxied without modification
  - Custom API endpoints for system cameras and status

Usage:
  python3 aprilvision-bridge.py [--port 5801] [--engine-port 5800]
"""

import http.server
import http.client
import socketserver
import socket
import select
import threading
import sys
import time
import gzip
import io
import re
import json
import os
import subprocess

# Configuration
PROXY_PORT = 5801
ENGINE_HOST = "127.0.0.1"
ENGINE_PORT = 5800

# ---------------------------------------------------------------------------
# Branding: text replacements applied to HTML, JS, CSS responses
# Order matters - more specific patterns first to avoid double-replacement
# ---------------------------------------------------------------------------
BRANDING_REPLACEMENTS = [
    # Exact product names
    (b"PhotonVision", b"AprilVision 3.2"),
    (b"Photon Vision", b"AprilVision 3.2"),
    (b"photon vision", b"AprilVision 3.2"),
    (b"PHOTONVISION", b"APRILVISION"),
    # Lowercase (CSS classes, paths in display text)
    (b"photonvision", b"aprilvision"),
    (b"photon-vision", b"april-vision"),
    # External links (block outgoing references)
    (b"https://photonvision.org", b"#"),
    (b"https://docs.photonvision.org", b"#"),
    (b"photonvision.org", b"aprilvision.local"),
    (b"docs.photonvision.org", b"aprilvision.local"),
    # GitHub references
    (b"PhotonVision/photonvision", b"AprilVision/aprilvision"),
    (b"github.com/PhotonVision", b"#"),
]

# ---------------------------------------------------------------------------
# CSS injection - custom branding overlay
# ---------------------------------------------------------------------------
CUSTOM_CSS = b"""
<style id="av-brand-css">
  /* AprilVision 3.2 - Team 1226 */
  /* Hide upstream logo/branding images */
  img[src*="photon"], img[src*="Photon"],
  img[alt*="PhotonVision"], img[alt*="photonvision"],
  svg[class*="photon"], [class*="photon-logo"],
  [class*="logo"] img[src*="photon"] {
    opacity: 0 !important;
    width: 0 !important;
    height: 0 !important;
    overflow: hidden !important;
  }
  /* Hide about/version dialogs that reference upstream */
  [class*="about"] [class*="photon"],
  [class*="version"] a[href*="photon"] {
    visibility: hidden !important;
  }
  /* Branding footer */
  body::after {
    content: "AprilVision 3.2 | Team 1226 | FRC 2026";
    position: fixed;
    bottom: 4px;
    right: 8px;
    font-size: 10px;
    color: rgba(0,255,136,0.5);
    z-index: 99999;
    pointer-events: none;
    font-family: -apple-system, BlinkMacSystemFont, sans-serif;
    letter-spacing: 0.3px;
  }
  /* Health indicator dot */
  body::before {
    content: "";
    position: fixed;
    bottom: 8px;
    left: 8px;
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: #00ff88;
    box-shadow: 0 0 4px rgba(0,255,136,0.5);
    z-index: 99999;
    pointer-events: none;
  }
  /* Custom scrollbar */
  ::-webkit-scrollbar { width: 8px; }
  ::-webkit-scrollbar-track { background: #1a1a2e; }
  ::-webkit-scrollbar-thumb { background: #00ff88; border-radius: 4px; }
</style>
"""

# ---------------------------------------------------------------------------
# JS injection - runtime DOM rebranding (handles SPA content)
# ---------------------------------------------------------------------------
CUSTOM_JS = b"""
<script id="av-rebrand-js">
(function() {
  var BRAND = 'AprilVision 3.2';
  var RE = [/PhotonVision/g, /Photon Vision/g, /photonvision/gi, /photon.vision/gi];

  function rebrand() {
    // Title
    if (document.title && !/AprilVision/.test(document.title)) {
      document.title = BRAND + ' Dashboard';
    }
    for (var i = 0; i < RE.length; i++) {
      if (RE[i].test(document.title)) {
        document.title = document.title.replace(RE[i], BRAND);
      }
    }

    // Text nodes
    var walker = document.createTreeWalker(
      document.body, NodeFilter.SHOW_TEXT, null, false
    );
    var node;
    while (node = walker.nextNode()) {
      if (node.nodeValue) {
        var orig = node.nodeValue;
        var val = orig;
        for (var i = 0; i < RE.length; i++) val = val.replace(RE[i], BRAND);
        if (val !== orig) node.nodeValue = val;
      }
    }

    // Attributes
    ['alt','title','placeholder','aria-label','content'].forEach(function(attr) {
      document.querySelectorAll('[' + attr + ']').forEach(function(el) {
        var val = el.getAttribute(attr);
        if (val) {
          var nv = val;
          for (var i = 0; i < RE.length; i++) nv = nv.replace(RE[i], BRAND);
          if (nv !== val) el.setAttribute(attr, nv);
        }
      });
    });

    // Hide logo images
    document.querySelectorAll('img').forEach(function(img) {
      var s = (img.src||'').toLowerCase(), a = (img.alt||'').toLowerCase();
      if (s.indexOf('photon') !== -1 || a.indexOf('photon') !== -1) {
        img.style.display = 'none';
      }
    });

    // Hide SVG logos
    document.querySelectorAll('svg').forEach(function(svg) {
      var cl = (svg.className && svg.className.baseVal) || '';
      if (cl.toLowerCase().indexOf('logo') !== -1 ||
          cl.toLowerCase().indexOf('photon') !== -1) {
        svg.style.display = 'none';
      }
    });

    // Block upstream links
    document.querySelectorAll('a').forEach(function(a) {
      if (a.href && (a.href.indexOf('photonvision.org') !== -1 ||
          a.href.indexOf('github.com/PhotonVision') !== -1)) {
        a.href = '#';
        a.onclick = function(e) { e.preventDefault(); };
      }
    });

    // Custom favicon
    var fav = document.querySelector("link[rel*='icon']");
    var ico = 'data:image/svg+xml,' + encodeURIComponent(
      '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">' +
      '<rect width="100" height="100" rx="20" fill="%230d1117"/>' +
      '<text x="50" y="68" font-size="55" text-anchor="middle" fill="%2300ff88" ' +
      'font-family="sans-serif" font-weight="bold">AV</text></svg>'
    );
    if (fav) { fav.href = ico; }
    else {
      fav = document.createElement('link');
      fav.rel = 'icon'; fav.href = ico;
      document.head.appendChild(fav);
    }
  }

  // Run on load
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', rebrand);
  } else { rebrand(); }

  // MutationObserver for SPA updates
  var obs = new MutationObserver(function() { rebrand(); });
  var startObs = function() {
    if (document.body) obs.observe(document.body, {childList:true,subtree:true,characterData:true});
  };
  if (document.body) startObs();
  else document.addEventListener('DOMContentLoaded', startObs);

  // Fallback periodic rebrand
  setInterval(rebrand, 3000);
})();
</script>
"""


# ---------------------------------------------------------------------------
# Branding application
# ---------------------------------------------------------------------------

def apply_branding(content, content_type):
    """Apply branding replacements and inject CSS/JS into responses."""
    if not content_type:
        return content

    ct = content_type.lower()

    # Only modify text-based content
    if "text/html" not in ct and "javascript" not in ct and "text/css" not in ct:
        return content

    modified = content
    for old, new in BRANDING_REPLACEMENTS:
        modified = modified.replace(old, new)

    # Inject custom CSS before </head> and JS before </body>
    if b"</head>" in modified or b"</body>" in modified:
        if b"</head>" in modified:
            modified = modified.replace(b"</head>", CUSTOM_CSS + b"</head>", 1)
        if b"</body>" in modified:
            modified = modified.replace(b"</body>", CUSTOM_JS + b"</body>", 1)

    return modified


# ---------------------------------------------------------------------------
# System camera enumeration
# ---------------------------------------------------------------------------

def get_system_cameras():
    """Enumerate cameras via V4L2."""
    cameras = []
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            current_name = None
            for line in lines:
                if not line.startswith('\t') and line.strip():
                    current_name = line.strip().rstrip(':')
                elif line.strip().startswith('/dev/video'):
                    cameras.append({
                        'name': current_name or 'Unknown',
                        'device': line.strip(),
                        'source': 'v4l2'
                    })
    except Exception:
        pass

    # Fallback: check /dev/video* directly
    import glob as _glob
    for dev in sorted(_glob.glob('/dev/video*')):
        if not any(c['device'] == dev for c in cameras):
            cameras.append({
                'name': 'Video Device',
                'device': dev,
                'source': 'devfs'
            })
    return cameras


# ---------------------------------------------------------------------------
# HTTP Reverse Proxy Handler
# ---------------------------------------------------------------------------

class ReverseProxyHandler(http.server.BaseHTTPRequestHandler):
    """Reverse proxy with WebSocket tunneling and branding injection."""

    def log_message(self, format, *args):
        pass  # Suppress default request logging

    def do_GET(self):
        # WebSocket upgrade -> tunnel
        if self._is_websocket_upgrade():
            self._tunnel_websocket()
            return
        self._proxy_request("GET")

    def do_POST(self):
        self._proxy_request("POST")

    def do_PUT(self):
        self._proxy_request("PUT")

    def do_DELETE(self):
        self._proxy_request("DELETE")

    def do_OPTIONS(self):
        self._proxy_request("OPTIONS")

    # --- WebSocket support --------------------------------------------------

    def _is_websocket_upgrade(self):
        """Detect WebSocket upgrade request."""
        upgrade = self.headers.get("Upgrade", "").lower()
        connection = self.headers.get("Connection", "").lower()
        return "websocket" in upgrade and "upgrade" in connection

    def _tunnel_websocket(self):
        """Tunnel WebSocket connection to the detection engine."""
        server_sock = None
        try:
            server_sock = socket.create_connection(
                (ENGINE_HOST, ENGINE_PORT), timeout=10
            )

            # Reconstruct and forward the HTTP upgrade request
            request_line = f"GET {self.path} HTTP/1.1\r\n"
            hdrs = ""
            for key in self.headers:
                for value in self.headers.get_all(key, []):
                    if key.lower() == "host":
                        hdrs += f"Host: {ENGINE_HOST}:{ENGINE_PORT}\r\n"
                    else:
                        hdrs += f"{key}: {value}\r\n"
            server_sock.sendall((request_line + hdrs + "\r\n").encode())

            client_sock = self.request

            # Bidirectional data relay
            sockets = [client_sock, server_sock]
            while True:
                readable, _, exceptional = select.select(sockets, [], sockets, 60)
                if exceptional or not readable:
                    break
                for sock in readable:
                    try:
                        data = sock.recv(65536)
                    except Exception:
                        return
                    if not data:
                        return
                    try:
                        target = server_sock if sock is client_sock else client_sock
                        target.sendall(data)
                    except Exception:
                        return
        except Exception as e:
            try:
                self._send_error_page(
                    502, "Connection Error",
                    f"Could not establish real-time connection: {e}"
                )
            except Exception:
                pass
        finally:
            if server_sock:
                try:
                    server_sock.close()
                except Exception:
                    pass

    # --- HTTP proxy ---------------------------------------------------------

    def _proxy_request(self, method):
        """Forward HTTP request to engine, rebrand response."""
        try:
            # Custom AprilVision API endpoints
            if self.path == '/api/aprilvision/cameras':
                return self._api_cameras()
            if self.path == '/api/aprilvision/status':
                return self._api_status()
            if self.path == '/api/aprilvision/snapshot' and method == 'POST':
                return self._api_snapshot()

            # Read request body
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else None

            # Connect to engine
            conn = http.client.HTTPConnection(
                ENGINE_HOST, ENGINE_PORT, timeout=30
            )

            # Forward headers
            headers = {}
            for key, value in self.headers.items():
                if key.lower() not in ("host", "accept-encoding"):
                    headers[key] = value
            headers["Host"] = f"{ENGINE_HOST}:{ENGINE_PORT}"
            headers["Accept-Encoding"] = "identity"

            conn.request(method, self.path, body=body, headers=headers)
            response = conn.getresponse()

            resp_body = response.read()
            content_type = response.getheader("Content-Type", "")

            # Decompress gzipped content
            content_encoding = response.getheader("Content-Encoding", "")
            if "gzip" in content_encoding.lower() and resp_body:
                try:
                    resp_body = gzip.decompress(resp_body)
                except Exception:
                    pass

            # Apply branding to text responses
            is_stream = (
                "multipart" in content_type.lower() or
                "octet-stream" in content_type.lower()
            )
            if not is_stream and resp_body:
                resp_body = apply_branding(resp_body, content_type)

            # Send response
            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() in (
                    "content-length", "transfer-encoding",
                    "content-encoding", "connection"
                ):
                    continue
                self.send_header(key, value)

            if resp_body is not None:
                self.send_header("Content-Length", str(len(resp_body)))
            self.end_headers()

            if resp_body:
                self.wfile.write(resp_body)

            conn.close()

        except ConnectionRefusedError:
            self._send_error_page(
                503, "Detection Engine Starting",
                "The AprilVision detection engine is starting up. "
                "This page will auto-refresh. If this persists, run: "
                "<code>./scripts/health_check.sh</code>"
            )
        except Exception as e:
            self._send_error_page(
                502, "Proxy Error",
                f"Connection to detection engine failed: {e}"
            )

    # --- Custom API endpoints -----------------------------------------------

    def _api_cameras(self):
        """Return cameras from V4L2 + engine."""
        cameras = get_system_cameras()

        # Also query the engine's camera API
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=3)
            conn.request("GET", "/api/settings/cameras")
            resp = conn.getresponse()
            if resp.status == 200:
                engine_data = json.loads(resp.read())
                for cam in (engine_data if isinstance(engine_data, list) else []):
                    cameras.append({
                        'name': cam.get('nickname', cam.get('uniqueName', 'Engine Camera')),
                        'device': cam.get('path', 'N/A'),
                        'source': 'engine',
                        'connected': True
                    })
            conn.close()
        except Exception:
            pass

        self._send_json({'cameras': cameras, 'count': len(cameras)})

    def _api_status(self):
        """Return system status."""
        status = {
            'version': 'AprilVision 3.2',
            'engine_online': False,
            'proxy_online': True,
            'cameras': get_system_cameras(),
        }
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=3)
            conn.request("GET", "/api/settings/general")
            resp = conn.getresponse()
            status['engine_online'] = resp.status == 200
            conn.close()
        except Exception:
            pass

        self._send_json(status)

    def _api_snapshot(self):
        """Save camera snapshot metadata for diagnostics."""
        snap_dir = "/opt/aprilvision/snapshots"
        os.makedirs(snap_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        saved = []

        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=5)
            conn.request("GET", "/api/settings/cameras")
            resp = conn.getresponse()
            if resp.status == 200:
                cams = json.loads(resp.read())
                for i, cam in enumerate(cams if isinstance(cams, list) else []):
                    name = cam.get('nickname', f'camera_{i}')
                    fpath = f"{snap_dir}/{ts}_{name}.json"
                    with open(fpath, 'w') as f:
                        f.write(json.dumps(cam, indent=2))
                    saved.append(fpath)
            conn.close()
        except Exception:
            pass

        self._send_json({'status': 'ok', 'timestamp': ts, 'files': saved})

    # --- Helpers ------------------------------------------------------------

    def _send_json(self, data):
        """Send a JSON response."""
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_error_page(self, code, title, message):
        """Send a branded error page."""
        body = f"""<!DOCTYPE html>
<html>
<head>
    <title>AprilVision 3.2 - {title}</title>
    <style>
        body {{
            background: #0d1117; color: #e6edf3;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
            display: flex; justify-content: center; align-items: center;
            height: 100vh; margin: 0;
        }}
        .error-box {{
            background: #161b22; border: 1px solid #30363d;
            border-radius: 12px; padding: 40px; max-width: 500px; text-align: center;
        }}
        h1 {{ color: #f85149; margin-bottom: 10px; }}
        h2 {{ color: #00ff88; margin-bottom: 20px; font-size: 1.1em; }}
        p {{ color: #8b949e; line-height: 1.6; }}
        .brand {{ color: #00ff88; font-size: 0.9em; margin-top: 20px; opacity: 0.7; }}
        code {{
            background: #0d1117; padding: 2px 8px;
            border-radius: 4px; font-size: 0.9em; color: #58a6ff;
        }}
    </style>
    <meta http-equiv="refresh" content="5">
</head>
<body>
    <div class="error-box">
        <h1>{code}</h1>
        <h2>{title}</h2>
        <p>{message}</p>
        <p style="color:#484f58;font-size:0.85em;">Auto-refreshing in 5 seconds...</p>
        <div class="brand">AprilVision 3.2 - FRC Competition Vision System</div>
    </div>
</body>
</html>""".encode("utf-8")

        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# MJPEG Streaming Handler
# ---------------------------------------------------------------------------

class StreamingProxyHandler(ReverseProxyHandler):
    """Extended handler with MJPEG streaming support."""

    def _proxy_request(self, method):
        """Override to handle streaming responses."""
        is_stream = (
            "/api/stream" in self.path or
            ".mjpeg" in self.path or
            "/stream" in self.path.lower()
        )

        if is_stream and method == "GET":
            self._proxy_stream()
        else:
            super()._proxy_request(method)

    def _proxy_stream(self):
        """Proxy streaming responses (MJPEG) without buffering."""
        try:
            conn = http.client.HTTPConnection(
                ENGINE_HOST, ENGINE_PORT, timeout=60
            )
            headers = {}
            for key, value in self.headers.items():
                if key.lower() != "host":
                    headers[key] = value
            headers["Host"] = f"{ENGINE_HOST}:{ENGINE_PORT}"

            conn.request("GET", self.path, headers=headers)
            response = conn.getresponse()

            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() != "transfer-encoding":
                    self.send_header(key, value)
            self.end_headers()

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


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------

class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Threaded HTTP server for concurrent request handling."""
    allow_reuse_address = True
    daemon_threads = True


def main():
    print("=" * 60)
    print("  AprilVision 3.2 - FRC Competition Vision System")
    print("  Built by Team 1226")
    print("=" * 60)
    print(f"  Dashboard:  http://0.0.0.0:{PROXY_PORT}")
    print(f"  Engine:     http://{ENGINE_HOST}:{ENGINE_PORT} (internal)")
    print()
    print("  WebSocket tunneling:  ENABLED")
    print("  Branding injection:   ACTIVE")
    print("  Camera API:           /api/aprilvision/cameras")
    print("  Status API:           /api/aprilvision/status")
    print("  Snapshot API:         /api/aprilvision/snapshot")
    print("=" * 60)
    print()

    server = ThreadedHTTPServer(
        ("0.0.0.0", PROXY_PORT), StreamingProxyHandler
    )
    print(f"[AprilVision] Proxy running on port {PROXY_PORT}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[AprilVision] Shutting down...")
        server.shutdown()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="AprilVision 3.2 Reverse Proxy")
    parser.add_argument(
        "--port", type=int, default=PROXY_PORT, help="Proxy port"
    )
    parser.add_argument(
        "--engine-port", type=int, default=ENGINE_PORT,
        help="Detection engine port"
    )
    # Backward compatibility
    parser.add_argument(
        "--pv-port", type=int, default=None,
        help="(deprecated, use --engine-port)"
    )
    args = parser.parse_args()
    PROXY_PORT = args.port
    ENGINE_PORT = args.engine_port if args.pv_port is None else args.pv_port
    main()
