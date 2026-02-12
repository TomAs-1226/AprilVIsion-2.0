#!/usr/bin/env python3
"""
AprilVision 3.2 - FRC Competition Vision System
Custom dashboard server with detection engine API proxy

Built by Team 1226.

Architecture:
  - Serves the custom AprilVision dashboard from the web/ directory
  - Proxies /api/* requests to the detection engine (port 5800)
  - Proxies /pv/* to the engine SPA with aggressive rebranding
  - Tunnels WebSocket connections for real-time camera data
  - Proxies camera MJPEG streams from engine stream ports (1181+)
  - Provides custom API endpoints at /api/av/*

Routing:
  /                    -> Custom dashboard (web/index.html)
  /app.js, /style.css  -> Static files from web/
  /api/av/*            -> Custom AprilVision API endpoints
  /api/*               -> Proxied to engine REST API
  /stream/<n>/<type>   -> Proxied MJPEG from engine camera ports
  /pv/*                -> Proxied engine SPA with rebranding (pipeline config)
  WebSocket upgrades   -> Tunneled to engine

Usage:
  python3 aprilvision-bridge.py [--port 5801] [--engine-port 5800]
"""

import http.server
import http.client
import socketserver
import socket
import select
import sys
import time
import gzip
import json
import os
import subprocess
import mimetypes
import urllib.parse

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PROXY_PORT = 5801
ENGINE_HOST = "127.0.0.1"
ENGINE_PORT = 5800
STREAM_BASE_PORT = 1181

# Path to custom dashboard files
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web")
# Also check the installed location
if not os.path.isdir(WEB_DIR):
    WEB_DIR = "/opt/aprilvision/web"

# ---------------------------------------------------------------------------
# Branding for engine SPA proxy (/pv/* route)
# ---------------------------------------------------------------------------
BRANDING_REPLACEMENTS = [
    (b"PhotonVision", b"AprilVision 3.2"),
    (b"Photon Vision", b"AprilVision 3.2"),
    (b"photon vision", b"AprilVision 3.2"),
    (b"PHOTONVISION", b"APRILVISION"),
    (b"photonvision", b"aprilvision"),
    (b"photon-vision", b"april-vision"),
    (b"https://photonvision.org", b"#"),
    (b"https://docs.photonvision.org", b"#"),
    (b"photonvision.org", b"aprilvision.local"),
    (b"docs.photonvision.org", b"aprilvision.local"),
    (b"PhotonVision/photonvision", b"AprilVision/aprilvision"),
    (b"github.com/PhotonVision", b"#"),
]

BRAND_CSS = b"""
<style id="av-css">
  img[src*="photon"],img[src*="Photon"],img[alt*="PhotonVision"],
  svg[class*="photon"],[class*="photon-logo"],[class*="logo"] img[src*="photon"]{
    opacity:0!important;width:0!important;height:0!important;overflow:hidden!important;
  }
  [class*="about"] [class*="photon"],[class*="version"] a[href*="photon"]{visibility:hidden!important;}
  body::after{content:"AprilVision 3.2 | Team 1226";position:fixed;bottom:4px;right:8px;
    font-size:10px;color:rgba(0,255,136,0.5);z-index:99999;pointer-events:none;
    font-family:-apple-system,sans-serif;letter-spacing:.3px;}
  ::-webkit-scrollbar{width:8px}::-webkit-scrollbar-track{background:#1a1a2e}
  ::-webkit-scrollbar-thumb{background:#00ff88;border-radius:4px}
</style>
"""

BRAND_JS = b"""
<script id="av-js">
(function(){
  var B='AprilVision 3.2',R=[/PhotonVision/g,/Photon Vision/g,/photonvision/gi];
  function go(){
    if(!/AprilVision/.test(document.title))document.title=B+' Dashboard';
    R.forEach(function(r){document.title=document.title.replace(r,B);});
    var w=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT,null,false),n;
    while(n=w.nextNode()){if(n.nodeValue){var o=n.nodeValue,v=o;
      R.forEach(function(r){v=v.replace(r,B);});if(v!==o)n.nodeValue=v;}}
    ['alt','title','placeholder','aria-label'].forEach(function(a){
      document.querySelectorAll('['+a+']').forEach(function(e){
        var v=e.getAttribute(a);if(v){var nv=v;
        R.forEach(function(r){nv=nv.replace(r,B);});
        if(nv!==v)e.setAttribute(a,nv);}});});
    document.querySelectorAll('img').forEach(function(i){
      if((i.src||'').toLowerCase().indexOf('photon')!==-1)i.style.display='none';});
    document.querySelectorAll('a').forEach(function(a){
      if(a.href&&(a.href.indexOf('photonvision.org')!==-1||
        a.href.indexOf('github.com/PhotonVision')!==-1)){a.href='#';
        a.onclick=function(e){e.preventDefault();};}});
    var f=document.querySelector("link[rel*='icon']");
    var ic='data:image/svg+xml,'+encodeURIComponent(
      '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">'+
      '<rect width="100" height="100" rx="20" fill="%230d1117"/>'+
      '<text x="50" y="68" font-size="55" text-anchor="middle" fill="%2300ff88" '+
      'font-family="sans-serif" font-weight="bold">AV</text></svg>');
    if(f){f.href=ic;}else{f=document.createElement('link');f.rel='icon';f.href=ic;
      document.head.appendChild(f);}
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',go);
  else go();
  var obs=new MutationObserver(go);
  var s=function(){if(document.body)obs.observe(document.body,
    {childList:true,subtree:true,characterData:true});};
  if(document.body)s();else document.addEventListener('DOMContentLoaded',s);
  setInterval(go,3000);
})();
</script>
"""


def apply_branding(content, content_type):
    """Apply branding to proxied engine SPA content."""
    if not content_type:
        return content
    ct = content_type.lower()
    if "text/html" not in ct and "javascript" not in ct and "text/css" not in ct:
        return content
    modified = content
    for old, new in BRANDING_REPLACEMENTS:
        modified = modified.replace(old, new)
    if b"</head>" in modified:
        modified = modified.replace(b"</head>", BRAND_CSS + b"</head>", 1)
    if b"</body>" in modified:
        modified = modified.replace(b"</body>", BRAND_JS + b"</body>", 1)
    return modified


def get_system_cameras():
    """Enumerate cameras via V4L2 and detection engine."""
    cameras = []
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            name = None
            for line in lines:
                if not line.startswith('\t') and line.strip():
                    name = line.strip().rstrip(':')
                elif line.strip().startswith('/dev/video'):
                    cameras.append({'name': name or 'Unknown', 'device': line.strip(), 'source': 'v4l2'})
    except Exception:
        pass
    import glob as _g
    for dev in sorted(_g.glob('/dev/video*')):
        if not any(c['device'] == dev for c in cameras):
            cameras.append({'name': 'Video Device', 'device': dev, 'source': 'devfs'})
    return cameras


# ---------------------------------------------------------------------------
# Request Handler
# ---------------------------------------------------------------------------

class DashboardHandler(http.server.BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass

    # --- HTTP methods -------------------------------------------------------

    def do_GET(self):
        if self._is_websocket_upgrade():
            return self._tunnel_websocket()
        self._route("GET")

    def do_POST(self):
        self._route("POST")

    def do_PUT(self):
        self._route("PUT")

    def do_DELETE(self):
        self._route("DELETE")

    def do_OPTIONS(self):
        self._route("OPTIONS")

    # --- Router -------------------------------------------------------------

    def _route(self, method):
        path = urllib.parse.urlparse(self.path).path

        # 1. Custom API
        if path.startswith('/api/av/'):
            return self._handle_custom_api(method, path)

        # 2. Camera stream proxy
        if path.startswith('/stream/'):
            return self._proxy_camera_stream(path)

        # 3. Engine SPA proxy (for pipeline config - with rebranding)
        if path.startswith('/pv/') or path.startswith('/pv'):
            return self._proxy_engine_spa(method)

        # 4. Engine API proxy (JSON, no branding needed)
        if path.startswith('/api/'):
            return self._proxy_engine_api(method)

        # 5. Static files from web/ directory
        if self._try_serve_static(path):
            return

        # 6. SPA catch-all: serve index.html for client-side routing
        self._serve_index()

    # --- Static file serving ------------------------------------------------

    def _try_serve_static(self, path):
        """Try to serve a static file from the web directory."""
        if path == '/':
            return False  # Let SPA catch-all handle root

        # Clean path to prevent directory traversal
        clean = os.path.normpath(path.lstrip('/'))
        if '..' in clean:
            return False

        filepath = os.path.join(WEB_DIR, clean)
        if os.path.isfile(filepath):
            self._send_file(filepath)
            return True
        return False

    def _serve_index(self):
        """Serve the main SPA index.html."""
        index_path = os.path.join(WEB_DIR, "index.html")
        if os.path.isfile(index_path):
            self._send_file(index_path)
        else:
            self._send_error(404, "Dashboard Not Found",
                             "Dashboard files not found. Run setup.sh to install.")

    def _send_file(self, filepath):
        """Send a static file with appropriate content type."""
        mime, _ = mimetypes.guess_type(filepath)
        if not mime:
            mime = 'application/octet-stream'
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(content)
        except Exception:
            self._send_error(500, "File Error", "Could not read file")

    # --- Camera stream proxy ------------------------------------------------

    def _proxy_camera_stream(self, path):
        """Proxy MJPEG stream from engine camera ports.
        /stream/<camera_index>          -> input stream (port 1181 + index*2)
        /stream/<camera_index>/input    -> input stream
        /stream/<camera_index>/output   -> output stream (port 1182 + index*2)
        """
        parts = path.strip('/').split('/')
        if len(parts) < 2:
            return self._send_error(400, "Bad Request", "Use /stream/&lt;camera_index&gt;")

        try:
            cam_idx = int(parts[1])
        except ValueError:
            return self._send_error(400, "Bad Request", "Camera index must be a number")

        stream_type = parts[2] if len(parts) > 2 else 'input'
        port = STREAM_BASE_PORT + (cam_idx * 2)
        if stream_type == 'output':
            port += 1

        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, port, timeout=10)
            conn.request("GET", "/")
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
            self._send_error(503, "Stream Unavailable",
                             f"Camera {cam_idx} stream not available on port {port}")

    # --- Engine API proxy ---------------------------------------------------

    def _proxy_engine_api(self, method):
        """Proxy REST API calls to the detection engine (no branding)."""
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else None

            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=15)
            headers = {}
            for key, value in self.headers.items():
                if key.lower() not in ("host", "accept-encoding"):
                    headers[key] = value
            headers["Host"] = f"{ENGINE_HOST}:{ENGINE_PORT}"

            conn.request(method, self.path, body=body, headers=headers)
            response = conn.getresponse()
            resp_body = response.read()

            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() in ("content-length", "transfer-encoding", "connection"):
                    continue
                self.send_header(key, value)
            self.send_header("Content-Length", str(len(resp_body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(resp_body)
            conn.close()
        except Exception as e:
            self._send_json({"error": str(e)}, 502)

    # --- Engine SPA proxy (rebranded, for pipeline config) ------------------

    def _proxy_engine_spa(self, method):
        """Proxy the engine's SPA with aggressive rebranding."""
        # Strip /pv prefix to get the real path
        real_path = self.path[3:] if self.path.startswith('/pv') else self.path
        if not real_path or real_path == '':
            real_path = '/'

        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else None

            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=30)
            headers = {}
            for key, value in self.headers.items():
                if key.lower() not in ("host", "accept-encoding"):
                    headers[key] = value
            headers["Host"] = f"{ENGINE_HOST}:{ENGINE_PORT}"
            headers["Accept-Encoding"] = "identity"

            conn.request(method, real_path, body=body, headers=headers)
            response = conn.getresponse()
            resp_body = response.read()
            content_type = response.getheader("Content-Type", "")

            # Decompress gzip
            enc = response.getheader("Content-Encoding", "")
            if "gzip" in enc.lower() and resp_body:
                try:
                    resp_body = gzip.decompress(resp_body)
                except Exception:
                    pass

            # Apply branding to text content
            is_binary = "multipart" in content_type.lower() or "octet-stream" in content_type.lower()
            if not is_binary and resp_body:
                resp_body = apply_branding(resp_body, content_type)

            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() in ("content-length", "transfer-encoding", "content-encoding", "connection"):
                    continue
                self.send_header(key, value)
            self.send_header("Content-Length", str(len(resp_body)))
            self.end_headers()
            self.wfile.write(resp_body)
            conn.close()

        except ConnectionRefusedError:
            self._send_error(503, "Engine Starting",
                             "Detection engine is starting up. Auto-refreshing...")
        except Exception as e:
            self._send_error(502, "Proxy Error", f"Engine connection failed: {e}")

    # --- WebSocket tunnel ---------------------------------------------------

    def _is_websocket_upgrade(self):
        return ("websocket" in self.headers.get("Upgrade", "").lower() and
                "upgrade" in self.headers.get("Connection", "").lower())

    def _tunnel_websocket(self):
        """Tunnel WebSocket to detection engine."""
        server_sock = None
        try:
            server_sock = socket.create_connection((ENGINE_HOST, ENGINE_PORT), timeout=10)
            req = f"GET {self.path} HTTP/1.1\r\n"
            for key in self.headers:
                for value in self.headers.get_all(key, []):
                    if key.lower() == "host":
                        req += f"Host: {ENGINE_HOST}:{ENGINE_PORT}\r\n"
                    else:
                        req += f"{key}: {value}\r\n"
            server_sock.sendall((req + "\r\n").encode())

            client_sock = self.request
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
                        (server_sock if sock is client_sock else client_sock).sendall(data)
                    except Exception:
                        return
        except Exception:
            pass
        finally:
            if server_sock:
                try:
                    server_sock.close()
                except Exception:
                    pass

    # --- Custom API ---------------------------------------------------------

    def _handle_custom_api(self, method, path):
        endpoint = path[len('/api/av/'):]

        if endpoint == 'cameras':
            return self._api_cameras()
        elif endpoint == 'status':
            return self._api_status()
        elif endpoint == 'snapshot' and method == 'POST':
            return self._api_snapshot()
        elif endpoint == 'streams':
            return self._api_streams()
        elif endpoint == 'engine-settings':
            return self._api_engine_settings()
        else:
            self._send_json({"error": "Unknown endpoint"}, 404)

    def _api_cameras(self):
        cameras = get_system_cameras()
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=3)
            conn.request("GET", "/api/settings")
            resp = conn.getresponse()
            if resp.status == 200:
                data = json.loads(resp.read())
                if 'cameraSettings' in data:
                    for i, cam in enumerate(data['cameraSettings']):
                        cameras.append({
                            'name': cam.get('nickname', cam.get('uniqueName', f'Camera {i}')),
                            'device': cam.get('path', 'N/A'),
                            'source': 'engine',
                            'index': i,
                            'connected': True,
                            'inputStreamPort': STREAM_BASE_PORT + (i * 2),
                            'outputStreamPort': STREAM_BASE_PORT + (i * 2) + 1,
                        })
            conn.close()
        except Exception:
            pass
        self._send_json({'cameras': cameras, 'count': len(cameras)})

    def _api_status(self):
        status = {
            'version': 'AprilVision 3.2',
            'engine_online': False,
            'proxy_online': True,
            'uptime': int(time.time() - START_TIME),
            'cameras': [],
        }
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=3)
            conn.request("GET", "/api/settings")
            resp = conn.getresponse()
            if resp.status == 200:
                status['engine_online'] = True
                data = json.loads(resp.read())
                if 'cameraSettings' in data:
                    for i, cam in enumerate(data['cameraSettings']):
                        calibrated = bool(cam.get('calibrations', []))
                        pipelines = cam.get('pipelineSettings', [])
                        current_pipe = cam.get('currentPipelineIndex', 0)
                        status['cameras'].append({
                            'name': cam.get('nickname', cam.get('uniqueName', f'Camera {i}')),
                            'index': i,
                            'calibrated': calibrated,
                            'pipelineCount': len(pipelines),
                            'currentPipeline': current_pipe,
                            'inputStreamUrl': f'/stream/{i}/input',
                            'outputStreamUrl': f'/stream/{i}/output',
                        })
            conn.close()
        except Exception:
            pass
        self._send_json(status)

    def _api_streams(self):
        """Return available camera stream URLs."""
        streams = []
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=3)
            conn.request("GET", "/api/settings")
            resp = conn.getresponse()
            if resp.status == 200:
                data = json.loads(resp.read())
                for i, cam in enumerate(data.get('cameraSettings', [])):
                    streams.append({
                        'camera': cam.get('nickname', f'Camera {i}'),
                        'index': i,
                        'inputUrl': f'/stream/{i}/input',
                        'outputUrl': f'/stream/{i}/output',
                        'inputPort': STREAM_BASE_PORT + (i * 2),
                        'outputPort': STREAM_BASE_PORT + (i * 2) + 1,
                    })
            conn.close()
        except Exception:
            pass
        self._send_json({'streams': streams})

    def _api_engine_settings(self):
        """Get detection engine settings (proxied)."""
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=5)
            conn.request("GET", "/api/settings")
            resp = conn.getresponse()
            if resp.status == 200:
                data = json.loads(resp.read())
                self._send_json(data)
            else:
                self._send_json({"error": "Engine returned " + str(resp.status)}, resp.status)
            conn.close()
        except Exception as e:
            self._send_json({"error": str(e)}, 502)

    def _api_snapshot(self):
        snap_dir = "/opt/aprilvision/snapshots"
        os.makedirs(snap_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        saved = []
        try:
            conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=5)
            conn.request("GET", "/api/settings")
            resp = conn.getresponse()
            if resp.status == 200:
                data = json.loads(resp.read())
                fpath = f"{snap_dir}/{ts}_settings.json"
                with open(fpath, 'w') as f:
                    f.write(json.dumps(data, indent=2))
                saved.append(fpath)
            conn.close()
        except Exception:
            pass
        self._send_json({'status': 'ok', 'timestamp': ts, 'files': saved})

    # --- Helpers ------------------------------------------------------------

    def _send_json(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, code, title, message):
        body = f"""<!DOCTYPE html><html><head>
<title>AprilVision 3.2 - {title}</title>
<style>body{{background:#0d1117;color:#e6edf3;font-family:-apple-system,sans-serif;
display:flex;justify-content:center;align-items:center;height:100vh;margin:0}}
.b{{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:40px;
max-width:500px;text-align:center}}h1{{color:#f85149}}h2{{color:#00ff88;font-size:1.1em}}
p{{color:#8b949e;line-height:1.6}}.f{{color:#00ff88;font-size:.9em;margin-top:20px;opacity:.7}}
code{{background:#0d1117;padding:2px 8px;border-radius:4px;color:#58a6ff}}</style>
<meta http-equiv="refresh" content="5"></head><body><div class="b">
<h1>{code}</h1><h2>{title}</h2><p>{message}</p>
<p style="color:#484f58;font-size:.85em">Auto-refreshing...</p>
<div class="f">AprilVision 3.2</div></div></body></html>""".encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------

class ThreadedServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

import gzip  # noqa: E402 (needed for SPA proxy)

START_TIME = time.time()


def main():
    print("=" * 60)
    print("  AprilVision 3.2 - FRC Competition Vision System")
    print("  Custom Dashboard Server - Built by Team 1226")
    print("=" * 60)
    print(f"  Dashboard:       http://0.0.0.0:{PROXY_PORT}")
    print(f"  Engine (hidden): http://{ENGINE_HOST}:{ENGINE_PORT}")
    print(f"  Web directory:   {WEB_DIR}")
    print()
    print("  Routes:")
    print(f"    /                -> Custom dashboard")
    print(f"    /api/av/*        -> AprilVision API")
    print(f"    /api/*           -> Engine API proxy")
    print(f"    /stream/<n>/*    -> Camera MJPEG proxy")
    print(f"    /pv/*            -> Engine SPA (rebranded)")
    print(f"    WebSocket        -> Tunneled to engine")
    print("=" * 60)

    server = ThreadedServer(("0.0.0.0", PROXY_PORT), DashboardHandler)
    print(f"[AprilVision] Dashboard running on port {PROXY_PORT}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[AprilVision] Shutting down...")
        server.shutdown()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="AprilVision 3.2 Dashboard Server")
    parser.add_argument("--port", type=int, default=PROXY_PORT)
    parser.add_argument("--engine-port", type=int, default=ENGINE_PORT)
    parser.add_argument("--pv-port", type=int, default=None, help="(deprecated)")
    args = parser.parse_args()
    PROXY_PORT = args.port
    ENGINE_PORT = args.engine_port if args.pv_port is None else args.pv_port
    main()
