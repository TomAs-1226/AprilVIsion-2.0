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
import threading

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PROXY_PORT = 5801
ENGINE_HOST = "127.0.0.1"
ENGINE_PORT = 5800
STREAM_BASE_PORT = 1181
TEAM_NUMBER = 0          # Set via --team; used for NT server address and dashboard display
NT_SERVER = "127.0.0.1"  # Overridden when --team is set (10.TE.AM.2)

# Path to custom dashboard files – resolved at import time, overridable via --web-dir
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_WEB_SEARCH_PATHS = [
    os.path.join(_SCRIPT_DIR, "..", "web"),           # repo: scripts/../web
    os.path.join(_SCRIPT_DIR, "web"),                 # scripts/web (flat layout)
    "/opt/aprilvision/web",                           # deployed location
    os.path.join(os.getcwd(), "web"),                 # cwd/web
]
WEB_DIR = None
for _p in _WEB_SEARCH_PATHS:
    _p = os.path.abspath(_p)
    if os.path.isdir(_p) and os.path.isfile(os.path.join(_p, "index.html")):
        WEB_DIR = _p
        break
if WEB_DIR is None:
    # Last resort: use the first candidate and let the 404 handler explain what's wrong
    WEB_DIR = os.path.abspath(_WEB_SEARCH_PATHS[0])

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
# Engine API Discovery
# ---------------------------------------------------------------------------
# PhotonVision's API has changed across versions.  We try multiple endpoints
# and response shapes so the dashboard works regardless of engine version.

# API paths to try in order of preference
_ENGINE_API_PATHS = [
    "/api/settings",                # PhotonVision v2024-v2025 (classic)
    "/api/v1/settings",             # Some v2026 builds
    "/api/settings/general",        # Alternate settings path
    "/",                            # Root may return JSON or HTML
]

# Keys under which camera settings may appear in the JSON response
_CAMERA_KEYS = [
    "cameraSettings",              # Classic PhotonVision
    "cameras",                     # Newer builds
    "cameraConfigurations",        # Some v2026 builds
    "configuredCameras",           # Alternate naming
]

def _engine_fetch(path, timeout=3):
    """Fetch a path from the engine and return (status, body_bytes)."""
    conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=timeout)
    conn.request("GET", path, headers={"Accept": "application/json"})
    resp = conn.getresponse()
    body = resp.read()
    status = resp.status
    conn.close()
    return status, body


def _engine_probe():
    """Try to contact the engine and return (online, settings_dict, api_path, error).

    Tries multiple API paths and parses JSON responses.  Returns the first
    successful result so the rest of the dashboard can use it.
    """
    last_error = None
    for api_path in _ENGINE_API_PATHS:
        try:
            status, body = _engine_fetch(api_path, timeout=3)
            if status == 200 and body:
                try:
                    data = json.loads(body)
                    if isinstance(data, dict):
                        return True, data, api_path, None
                except json.JSONDecodeError:
                    # 200 but not JSON (probably the HTML SPA) – engine is
                    # up, but this isn't the settings endpoint.
                    last_error = f"{api_path} returned non-JSON (HTML?)"
                    continue
            else:
                last_error = f"{api_path} returned HTTP {status}"
        except socket.timeout:
            last_error = f"{api_path} timed out"
        except ConnectionRefusedError:
            last_error = f"Connection refused on {ENGINE_HOST}:{ENGINE_PORT}"
            break  # No point trying other paths if port is closed
        except Exception as e:
            last_error = f"{api_path}: {e}"

    # One more check: even if settings API failed, see if engine port is open
    try:
        s = socket.create_connection((ENGINE_HOST, ENGINE_PORT), timeout=2)
        s.close()
        return True, {}, None, last_error or "Engine reachable but settings API unavailable"
    except Exception:
        pass

    return False, {}, None, last_error or f"Cannot reach engine at {ENGINE_HOST}:{ENGINE_PORT}"


def _extract_cameras(data):
    """Extract camera list from engine settings dict regardless of key naming."""
    for key in _CAMERA_KEYS:
        val = data.get(key)
        if val:
            if isinstance(val, list):
                return val
            if isinstance(val, dict):
                # dict keyed by camera name/id
                return list(val.values())

    # Deep search: look for any list of dicts that have camera-like keys
    camera_indicators = {"uniqueName", "nickname", "path", "cameraPath", "name"}
    for key, val in data.items():
        if isinstance(val, list) and len(val) > 0 and isinstance(val[0], dict):
            item_keys = set(val[0].keys())
            if item_keys & camera_indicators:
                return val
    return []


# ---------------------------------------------------------------------------
# Background Target/Pose Data Poller
# ---------------------------------------------------------------------------
# Reads live detection results from NetworkTables (if ntcore available) or
# by polling the engine's API.  Caches the latest state for API consumers.

_latest_targets = {
    "cameras": {},        # {cam_index: {targets: [...], latency: ..., fps: ...}}
    "robot_pose": None,   # {x, y, z, rx, ry, rz} in field-space
    "last_update": 0,
    "nt_connected": False,
    "source": "none",     # "ntcore", "api", or "none"
}
_targets_lock = threading.Lock()


def _start_target_poller():
    """Start background thread to collect target/pose data."""
    t = threading.Thread(target=_target_poll_loop, daemon=True)
    t.start()


def _target_poll_loop():
    """Continuously poll for target data from engine or NT."""
    # Try ntcore first
    nt_client = _try_ntcore()
    if nt_client:
        _poll_ntcore(nt_client)
    else:
        _poll_api_targets()


def _try_ntcore():
    """Try to import and initialize ntcore for NetworkTables data.

    PhotonVision runs its own NT server (runNTServer: true), so we always
    connect to localhost first.  When a roboRIO is also present the PV
    engine bridges the data automatically.
    """
    try:
        import ntcore
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("AprilVision Dashboard")
        # Always connect to local PV NT server first (port 5810 for NT4)
        inst.setServer("127.0.0.1", ntcore.NetworkTableInstance.kDefaultPort4)
        with _targets_lock:
            _latest_targets["source"] = "ntcore"
        return inst
    except ImportError:
        pass
    try:
        from networktables import NetworkTables
        NetworkTables.initialize(server="127.0.0.1")
        with _targets_lock:
            _latest_targets["source"] = "ntcore"
        return NetworkTables
    except ImportError:
        pass
    return None


def _poll_ntcore(nt):
    """Poll NetworkTables for PhotonVision target data."""
    while True:
        try:
            connected = False
            try:
                connected = nt.isConnected()
            except AttributeError:
                try:
                    connected = nt.isConnected()
                except Exception:
                    pass

            pv_table = None
            try:
                pv_table = nt.getTable("photonvision")
            except AttributeError:
                try:
                    pv_table = nt.getTable("/photonvision")
                except Exception:
                    pass

            with _targets_lock:
                _latest_targets["nt_connected"] = connected
                _latest_targets["last_update"] = time.time()

                if pv_table:
                    subtables = []
                    try:
                        subtables = pv_table.getSubTables()
                    except Exception:
                        pass

                    for cam_name in subtables:
                        try:
                            cam_table = pv_table.getSubTable(cam_name)
                            has_target = cam_table.getEntry("hasTarget").getBoolean(False)
                            latency = cam_table.getEntry("latencyMillis").getDouble(0)
                            target_yaw = cam_table.getEntry("targetYaw").getDouble(0)
                            target_pitch = cam_table.getEntry("targetPitch").getDouble(0)
                            target_area = cam_table.getEntry("targetArea").getDouble(0)
                            target_skew = cam_table.getEntry("targetSkew").getDouble(0)
                            best_id = cam_table.getEntry("targetFiducialId").getDouble(-1)

                            # 3D pose if available
                            pose_data = None
                            try:
                                bp = cam_table.getEntry("targetPose")
                                pose_arr = bp.getDoubleArray([])
                                if len(pose_arr) >= 3:
                                    pose_data = {
                                        "x": round(pose_arr[0], 4),
                                        "y": round(pose_arr[1], 4),
                                        "z": round(pose_arr[2], 4),
                                        "rx": round(pose_arr[3], 2) if len(pose_arr) > 3 else 0,
                                        "ry": round(pose_arr[4], 2) if len(pose_arr) > 4 else 0,
                                        "rz": round(pose_arr[5], 2) if len(pose_arr) > 5 else 0,
                                    }
                            except Exception:
                                pass

                            _latest_targets["cameras"][cam_name] = {
                                "hasTarget": has_target,
                                "latencyMs": round(latency, 1),
                                "targetYaw": round(target_yaw, 2),
                                "targetPitch": round(target_pitch, 2),
                                "targetArea": round(target_area, 2),
                                "targetSkew": round(target_skew, 2),
                                "bestFiducialId": int(best_id),
                                "pose": pose_data,
                            }
                        except Exception:
                            pass

                    # Robot pose from multi-tag (if published by robot code)
                    try:
                        rp = nt.getTable("SmartDashboard").getSubTable("AprilVision")
                        pose_x = rp.getEntry("poseX").getDouble(0)
                        pose_y = rp.getEntry("poseY").getDouble(0)
                        pose_rot = rp.getEntry("poseRotation").getDouble(0)
                        if pose_x != 0 or pose_y != 0:
                            _latest_targets["robot_pose"] = {
                                "x": round(pose_x, 4),
                                "y": round(pose_y, 4),
                                "z": 0,
                                "rotation": round(pose_rot, 2),
                            }
                    except Exception:
                        pass
        except Exception:
            pass
        time.sleep(0.1)


def _poll_api_targets():
    """Fallback: poll engine HTTP API for any available target data."""
    with _targets_lock:
        _latest_targets["source"] = "api"
    while True:
        try:
            online, data, _, _ = _engine_probe()
            if online and data:
                cam_list = _extract_cameras(data)
                with _targets_lock:
                    _latest_targets["last_update"] = time.time()
                    for i, cam in enumerate(cam_list):
                        cam_name = cam.get('nickname', cam.get('uniqueName', f'Camera_{i}'))
                        # Extract any result data available in the settings response
                        current_pipe_idx = cam.get('currentPipelineIndex', 0)
                        pipelines = cam.get('pipelineSettings', cam.get('pipelines', []))
                        if isinstance(pipelines, list) and pipelines:
                            pipe = pipelines[min(current_pipe_idx, len(pipelines) - 1)] if isinstance(current_pipe_idx, int) else pipelines[0]
                            if isinstance(pipe, dict):
                                _latest_targets["cameras"][cam_name] = {
                                    "hasTarget": False,
                                    "latencyMs": 0,
                                    "pipelineType": pipe.get("pipelineType", "unknown"),
                                    "pipelineName": pipe.get("pipelineNickname", pipe.get("name", f"Pipeline {current_pipe_idx}")),
                                    "cameraIndex": i,
                                }
        except Exception:
            pass
        time.sleep(1)


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
                             f"index.html not found in <code>{WEB_DIR}</code>. "
                             f"Run <code>deploy.sh</code> or pass <code>--web-dir /path/to/web</code>.")

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
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(content)
        except Exception:
            self._send_error(500, "File Error", "Could not read file")

    # --- Camera stream proxy ------------------------------------------------

    def _proxy_camera_stream(self, path):
        """Proxy MJPEG stream from engine camera ports using raw sockets.

        Uses raw socket relay instead of http.client because MJPEG streams
        are multipart/x-mixed-replace and http.client.read(n) blocks until
        it fills the buffer, killing real-time frame delivery.

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

        sock = None
        # Try multiple paths that cscore MJPEG servers commonly respond to
        stream_paths = ["/stream.mjpg", "/?action=stream", "/"]
        try:
            sock = socket.create_connection((ENGINE_HOST, port), timeout=5)
            sock.settimeout(5)

            # Try each path until we get an MJPEG response
            connected = False
            for try_path in stream_paths:
                try:
                    request = (
                        f"GET {try_path} HTTP/1.0\r\n"
                        f"Host: {ENGINE_HOST}:{port}\r\n"
                        f"Connection: close\r\n"
                        f"\r\n"
                    )
                    sock.sendall(request.encode())

                    # Read HTTP response headers
                    header_buf = b""
                    while b"\r\n\r\n" not in header_buf:
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionError("Connection closed reading headers")
                        header_buf += chunk
                        if len(header_buf) > 65536:
                            raise ConnectionError("Header too large")

                    header_end = header_buf.index(b"\r\n\r\n") + 4
                    headers_raw = header_buf[:header_end]
                    body_start = header_buf[header_end:]

                    # Parse status line
                    status_line = headers_raw.split(b"\r\n")[0]
                    status_parts = status_line.split(b" ", 2)
                    status_code = int(status_parts[1])

                    if status_code == 200:
                        connected = True
                        break

                    # Non-200: reconnect and try next path
                    sock.close()
                    sock = socket.create_connection((ENGINE_HOST, port), timeout=5)
                    sock.settimeout(5)
                except (socket.timeout, ConnectionError):
                    if try_path != stream_paths[-1]:
                        try:
                            sock.close()
                        except Exception:
                            pass
                        sock = socket.create_connection((ENGINE_HOST, port), timeout=5)
                        sock.settimeout(5)
                        continue
                    raise

            if not connected:
                raise ConnectionError("No valid MJPEG path found")

            # Send response headers to client
            self.send_response(status_code)
            for header_line in headers_raw.split(b"\r\n")[1:]:
                if not header_line:
                    continue
                if b":" in header_line:
                    key, value = header_line.split(b":", 1)
                    key_str = key.strip().decode("ascii", errors="replace")
                    val_str = value.strip().decode("ascii", errors="replace")
                    if key_str.lower() in ("transfer-encoding", "connection"):
                        continue
                    self.send_header(key_str, val_str)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            # Send any body data that arrived with the headers
            if body_start:
                self.wfile.write(body_start)
                self.wfile.flush()

            # Relay MJPEG stream data using select() for non-blocking reads
            sock.settimeout(0)
            while True:
                readable, _, _ = select.select([sock], [], [], 30)
                if not readable:
                    break  # 30s timeout with no data
                data = sock.recv(65536)
                if not data:
                    break
                self.wfile.write(data)
                self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            try:
                self._send_error(503, "Stream Unavailable",
                                 f"Camera {cam_idx} {stream_type} stream not available on port {port}")
            except Exception:
                pass
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

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
        elif endpoint == 'stream-health':
            return self._api_stream_health()
        elif endpoint == 'camera-info':
            return self._api_camera_info()
        elif endpoint == 'engine-settings':
            return self._api_engine_settings()
        elif endpoint == 'engine-debug':
            return self._api_engine_debug()
        elif endpoint == 'targets':
            return self._api_targets()
        elif endpoint == 'match-readiness':
            return self._api_match_readiness()
        else:
            self._send_json({"error": "Unknown endpoint"}, 404)

    def _api_cameras(self):
        system_cameras = get_system_cameras()
        engine_cameras = []
        online, data, api_path, err = _engine_probe()
        if online and data:
            for i, cam in enumerate(_extract_cameras(data)):
                engine_cameras.append({
                    'name': cam.get('nickname', cam.get('uniqueName', cam.get('name', f'Camera {i}'))),
                    'device': cam.get('path', cam.get('cameraPath', 'N/A')),
                    'source': 'engine',
                    'index': i,
                    'connected': True,
                    'inputStreamPort': STREAM_BASE_PORT + (i * 2),
                    'outputStreamPort': STREAM_BASE_PORT + (i * 2) + 1,
                })
        all_cameras = system_cameras + engine_cameras
        self._send_json({
            'cameras': all_cameras,
            'count': len(all_cameras),
            'engine_online': online,
            'engine_error': err,
        })

    def _api_status(self):
        online, data, api_path, err = _engine_probe()
        cam_list = _extract_cameras(data) if data else []
        cameras_out = []
        for i, cam in enumerate(cam_list):
            calibrations = cam.get('calibrations', cam.get('cameraCalibrations', []))
            calibrated = bool(calibrations) if isinstance(calibrations, list) else bool(calibrations)
            pipelines = cam.get('pipelineSettings', cam.get('pipelines', []))
            if isinstance(pipelines, list):
                pipe_count = len(pipelines)
            elif isinstance(pipelines, dict):
                pipe_count = len(pipelines)
            else:
                pipe_count = 0
            current_pipe = cam.get('currentPipelineIndex', cam.get('pipelineIndex', 0))
            cameras_out.append({
                'name': cam.get('nickname', cam.get('uniqueName', cam.get('name', f'Camera {i}'))),
                'index': i,
                'calibrated': calibrated,
                'pipelineCount': pipe_count,
                'currentPipeline': current_pipe,
                'inputStreamUrl': f'/stream/{i}/input',
                'outputStreamUrl': f'/stream/{i}/output',
            })
        self._send_json({
            'version': 'AprilVision 3.2',
            'engine_online': online,
            'engine_error': err,
            'engine_api_path': api_path,
            'proxy_online': True,
            'uptime': int(time.time() - START_TIME),
            'cameras': cameras_out,
            'camera_count': len(cameras_out),
            'team_number': TEAM_NUMBER if TEAM_NUMBER > 0 else None,
            'nt_server': NT_SERVER,
        })

    def _api_streams(self):
        """Return available camera stream URLs."""
        online, data, _, err = _engine_probe()
        cam_list = _extract_cameras(data) if data else []
        streams = []
        for i, cam in enumerate(cam_list):
            streams.append({
                'camera': cam.get('nickname', cam.get('name', f'Camera {i}')),
                'index': i,
                'inputUrl': f'/stream/{i}/input',
                'outputUrl': f'/stream/{i}/output',
                'inputPort': STREAM_BASE_PORT + (i * 2),
                'outputPort': STREAM_BASE_PORT + (i * 2) + 1,
            })
        self._send_json({'streams': streams, 'engine_online': online, 'engine_error': err})

    def _api_engine_debug(self):
        """Raw diagnostic: shows exactly what the engine returns for debugging."""
        results = {
            'engine_host': ENGINE_HOST,
            'engine_port': ENGINE_PORT,
            'stream_base_port': STREAM_BASE_PORT,
            'probes': [],
        }
        # Probe the engine port
        try:
            s = socket.create_connection((ENGINE_HOST, ENGINE_PORT), timeout=2)
            s.close()
            results['port_open'] = True
        except Exception as e:
            results['port_open'] = False
            results['port_error'] = str(e)

        # Try each API path and record what comes back
        for api_path in _ENGINE_API_PATHS:
            probe = {'path': api_path}
            try:
                status, body = _engine_fetch(api_path, timeout=3)
                probe['status'] = status
                probe['body_length'] = len(body)
                probe['content_preview'] = body[:2000].decode('utf-8', errors='replace')
                try:
                    parsed = json.loads(body)
                    probe['is_json'] = True
                    probe['top_keys'] = list(parsed.keys()) if isinstance(parsed, dict) else f'[array of {len(parsed)}]'
                    # Find camera data
                    cams = _extract_cameras(parsed) if isinstance(parsed, dict) else []
                    probe['cameras_found'] = len(cams)
                    if cams:
                        probe['first_camera_keys'] = list(cams[0].keys()) if isinstance(cams[0], dict) else str(type(cams[0]))
                except json.JSONDecodeError:
                    probe['is_json'] = False
            except Exception as e:
                probe['error'] = str(e)
            results['probes'].append(probe)

        # Check stream ports for first 4 possible cameras
        results['stream_probes'] = []
        for i in range(4):
            port = STREAM_BASE_PORT + (i * 2)
            results['stream_probes'].append({
                'camera_index': i,
                'input_port': port,
                'output_port': port + 1,
                'input_status': self._check_stream_port(port),
                'output_status': self._check_stream_port(port + 1),
            })

        self._send_json(results)

    def _check_stream_port(self, port):
        """Check if a stream port is reachable and serving MJPEG."""
        try:
            s = socket.create_connection((ENGINE_HOST, port), timeout=2)
            s.sendall(f"GET /stream.mjpg HTTP/1.0\r\nHost: {ENGINE_HOST}:{port}\r\n\r\n".encode())
            s.settimeout(2)
            data = s.recv(512)
            s.close()
            if b"200" in data and (b"multipart" in data.lower() or b"image" in data.lower()):
                return "live"
            if b"200" in data:
                return "responding"
            return "error"
        except (socket.timeout, ConnectionRefusedError, OSError):
            return "unreachable"
        except Exception:
            return "error"

    def _api_stream_health(self):
        """Check availability of all camera stream ports."""
        results = []
        online, data, _, err = _engine_probe()
        cam_list = _extract_cameras(data) if data else []

        if cam_list:
            for i, cam in enumerate(cam_list):
                input_port = STREAM_BASE_PORT + (i * 2)
                output_port = input_port + 1
                results.append({
                    'camera': cam.get('nickname', cam.get('uniqueName', cam.get('name', f'Camera {i}'))),
                    'index': i,
                    'inputPort': input_port,
                    'outputPort': output_port,
                    'inputStatus': self._check_stream_port(input_port),
                    'outputStatus': self._check_stream_port(output_port),
                    'inputUrl': f'/stream/{i}/input',
                    'outputUrl': f'/stream/{i}/output',
                })
        else:
            # No cameras found in engine config - still probe the first few ports
            # in case streams are running even though settings API differs
            for i in range(4):
                input_port = STREAM_BASE_PORT + (i * 2)
                output_port = input_port + 1
                input_status = self._check_stream_port(input_port)
                output_status = self._check_stream_port(output_port)
                if input_status != 'unreachable' or output_status != 'unreachable':
                    results.append({
                        'camera': f'Camera {i} (discovered)',
                        'index': i,
                        'inputPort': input_port,
                        'outputPort': output_port,
                        'inputStatus': input_status,
                        'outputStatus': output_status,
                        'inputUrl': f'/stream/{i}/input',
                        'outputUrl': f'/stream/{i}/output',
                    })

        self._send_json({
            'streams': results,
            'engine': ENGINE_HOST,
            'engine_online': online,
            'engine_error': err,
        })

    def _api_camera_info(self):
        """Rich camera information combining system devices and engine data."""
        system_cameras = get_system_cameras()
        engine_cameras = []
        online, data, api_path, err = _engine_probe()
        cam_list = _extract_cameras(data) if data else []

        for i, cam in enumerate(cam_list):
            input_port = STREAM_BASE_PORT + (i * 2)
            output_port = input_port + 1
            calibrations = cam.get('calibrations', cam.get('cameraCalibrations', []))
            if not isinstance(calibrations, list):
                calibrations = []
            resolutions = []
            for cal in calibrations:
                r = cal.get('resolution', {})
                if r:
                    resolutions.append(f"{r.get('width', '?')}x{r.get('height', '?')}")

            # Current pipeline info
            pipelines = cam.get('pipelineSettings', cam.get('pipelines', []))
            if not isinstance(pipelines, list):
                pipelines = list(pipelines.values()) if isinstance(pipelines, dict) else []
            current_idx = cam.get('currentPipelineIndex', cam.get('pipelineIndex', 0))
            current_pipeline = None
            if pipelines and isinstance(current_idx, int) and current_idx < len(pipelines):
                cp = pipelines[current_idx]
                if isinstance(cp, dict):
                    current_pipeline = {
                        'index': current_idx,
                        'name': cp.get('pipelineNickname', cp.get('name', f'Pipeline {current_idx}')),
                        'type': cp.get('pipelineType', cp.get('type', 'unknown')),
                    }

            engine_cameras.append({
                'name': cam.get('nickname', cam.get('uniqueName', cam.get('name', f'Camera {i}'))),
                'uniqueName': cam.get('uniqueName', cam.get('name', '')),
                'devicePath': cam.get('path', cam.get('cameraPath', 'N/A')),
                'index': i,
                'calibrated': bool(calibrations),
                'calibrationCount': len(calibrations),
                'calibratedResolutions': resolutions,
                'pipelineCount': len(pipelines),
                'currentPipeline': current_pipeline,
                'inputStreamPort': input_port,
                'outputStreamPort': output_port,
                'inputStreamUrl': f'/stream/{i}/input',
                'outputStreamUrl': f'/stream/{i}/output',
                'directInputUrl': f'http://{ENGINE_HOST}:{input_port}/stream.mjpg',
                'directOutputUrl': f'http://{ENGINE_HOST}:{output_port}/stream.mjpg',
                'inputStreamStatus': self._check_stream_port(input_port),
                'outputStreamStatus': self._check_stream_port(output_port),
            })

        self._send_json({
            'systemCameras': system_cameras,
            'engineCameras': engine_cameras,
            'engineHost': ENGINE_HOST,
            'enginePort': ENGINE_PORT,
            'streamBasePort': STREAM_BASE_PORT,
            'engineOnline': online,
            'engineApiPath': api_path,
            'engineError': err,
        })

    def _api_targets(self):
        """Return latest target/pose data from NT or engine API."""
        with _targets_lock:
            data = {
                "cameras": dict(_latest_targets["cameras"]),
                "robotPose": _latest_targets["robot_pose"],
                "lastUpdate": _latest_targets["last_update"],
                "ntConnected": _latest_targets["nt_connected"],
                "dataSource": _latest_targets["source"],
                "age": round(time.time() - _latest_targets["last_update"], 1)
                       if _latest_targets["last_update"] else None,
            }
        self._send_json(data)

    def _api_match_readiness(self):
        """Comprehensive pre-match system check."""
        checks = []
        online, data, api_path, err = _engine_probe()

        # 1. Engine reachable
        checks.append({
            'name': 'Detection Engine',
            'pass': online,
            'detail': f'Online via {api_path}' if online else (err or 'Unreachable'),
            'critical': True,
        })

        # 2. Cameras detected
        cam_list = _extract_cameras(data) if data else []
        checks.append({
            'name': 'Cameras Connected',
            'pass': len(cam_list) > 0,
            'detail': f'{len(cam_list)} camera(s) found' if cam_list else 'No cameras in engine config',
            'critical': True,
        })

        # 3. Calibration
        calibrated_count = 0
        for cam in cam_list:
            cals = cam.get('calibrations', cam.get('cameraCalibrations', []))
            if cals:
                calibrated_count += 1
        checks.append({
            'name': 'Camera Calibration',
            'pass': calibrated_count == len(cam_list) and len(cam_list) > 0,
            'detail': f'{calibrated_count}/{len(cam_list)} calibrated',
            'critical': False,
        })

        # 4. Streams alive
        streams_live = 0
        for i in range(len(cam_list)):
            port = STREAM_BASE_PORT + (i * 2) + 1  # output stream
            if self._check_stream_port(port) == 'live':
                streams_live += 1
        checks.append({
            'name': 'Camera Streams',
            'pass': streams_live == len(cam_list) and len(cam_list) > 0,
            'detail': f'{streams_live}/{len(cam_list)} streams live',
            'critical': True,
        })

        # 5. NetworkTables
        with _targets_lock:
            nt_ok = _latest_targets["nt_connected"]
            nt_source = _latest_targets["source"]
        checks.append({
            'name': 'NetworkTables',
            'pass': nt_ok,
            'detail': f'Connected (via {nt_source})' if nt_ok else f'Not connected (source: {nt_source})',
            'critical': False,
        })

        # 6. Target data flowing
        with _targets_lock:
            has_targets = any(
                c.get("hasTarget", False) for c in _latest_targets["cameras"].values()
            )
            data_age = time.time() - _latest_targets["last_update"] if _latest_targets["last_update"] else 999
        checks.append({
            'name': 'Target Data',
            'pass': data_age < 5,
            'detail': f'Data is {round(data_age, 1)}s old' + (' (targets visible)' if has_targets else ' (no targets in view)'),
            'critical': False,
        })

        all_critical = all(c['pass'] for c in checks if c['critical'])
        all_pass = all(c['pass'] for c in checks)
        self._send_json({
            'checks': checks,
            'ready': all_critical,
            'allPass': all_pass,
            'summary': 'MATCH READY' if all_critical else 'NOT READY - critical checks failed',
        })

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

START_TIME = time.time()


def main():
    print("=" * 60)
    print("  AprilVision 3.2 - FRC Competition Vision System")
    print("  Custom Dashboard Server - Built by Team 1226")
    print("=" * 60)
    print(f"  Dashboard:       http://0.0.0.0:{PROXY_PORT}")
    print(f"  Engine (hidden): http://{ENGINE_HOST}:{ENGINE_PORT}")
    if TEAM_NUMBER > 0:
        print(f"  Team Number:     {TEAM_NUMBER}")
        print(f"  NT Server:       {NT_SERVER}")
    else:
        print(f"  Team Number:     (not set - use --team)")
        print(f"  NT Server:       {NT_SERVER} (local)")
    print(f"  Web directory:   {WEB_DIR}")
    index_ok = os.path.isfile(os.path.join(WEB_DIR, "index.html"))
    if index_ok:
        print(f"  Dashboard files: OK (index.html found)")
    else:
        print(f"  WARNING: index.html NOT FOUND in {WEB_DIR}")
        print(f"  Run: sudo ./deploy.sh  or pass --web-dir /path/to/web")
    print()
    print("  Routes:")
    print(f"    /                -> Custom dashboard")
    print(f"    /api/av/*        -> AprilVision API")
    print(f"    /api/*           -> Engine API proxy")
    print(f"    /stream/<n>/*    -> Camera MJPEG proxy")
    print(f"    /pv/*            -> Engine SPA (rebranded)")
    print(f"    WebSocket        -> Tunneled to engine")
    print("=" * 60)

    _start_target_poller()
    print("[AprilVision] Target data poller started")

    server = ThreadedServer(("0.0.0.0", PROXY_PORT), DashboardHandler)
    print(f"[AprilVision] Dashboard running on port {PROXY_PORT}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[AprilVision] Shutting down...")
        server.shutdown()


def _team_to_ip(team):
    """Convert FRC team number to roboRIO IP (10.TE.AM.2)."""
    if team <= 0:
        return "127.0.0.1"
    t = str(team)
    if len(t) <= 2:
        return f"10.0.{t}.2"
    elif len(t) == 3:
        return f"10.{t[0]}.{t[1:]}.2"
    else:
        return f"10.{t[:2]}.{t[2:]}.2"


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="AprilVision 3.2 Dashboard Server")
    parser.add_argument("--port", type=int, default=PROXY_PORT)
    parser.add_argument("--engine-port", type=int, default=ENGINE_PORT)
    parser.add_argument("--team", type=int, default=0,
                        help="FRC team number (sets NT server to 10.TE.AM.2)")
    parser.add_argument("--nt-server", type=str, default=None,
                        help="NetworkTables server address (overrides --team)")
    parser.add_argument("--web-dir", type=str, default=None,
                        help="Path to the web/ directory containing index.html")
    parser.add_argument("--pv-port", type=int, default=None, help="(deprecated)")
    args = parser.parse_args()
    PROXY_PORT = args.port
    ENGINE_PORT = args.engine_port if args.pv_port is None else args.pv_port
    if args.team > 0:
        TEAM_NUMBER = args.team
        NT_SERVER = _team_to_ip(args.team)
    if args.nt_server:
        NT_SERVER = args.nt_server
    if args.web_dir:
        WEB_DIR = os.path.abspath(args.web_dir)
    main()
