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
import struct
import hashlib
import base64
import io

# Optional: WebSocket client + MessagePack for live PV data
try:
    import websocket as _ws_lib
    _HAS_WEBSOCKET = True
except ImportError:
    _HAS_WEBSOCKET = False

try:
    import msgpack
    _HAS_MSGPACK = True
except ImportError:
    _HAS_MSGPACK = False

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

    # Nested settings payloads (seen in some 2026+ builds)
    for nested_key in ("settings", "generalSettings", "state", "config"):
        nested = data.get(nested_key)
        if isinstance(nested, dict):
            nested_cams = _extract_cameras(nested)
            if nested_cams:
                return nested_cams
    return []


def _stream_ports_for_camera(cam_idx, cam_obj=None):
    """Return (input_port, output_port) for a camera index.

    Prefers per-camera ports published by PhotonVision (WS/settings), falling
    back to the historical base-port convention.
    """
    default_input = STREAM_BASE_PORT + (cam_idx * 2)
    default_output = default_input + 1

    if isinstance(cam_obj, dict):
        in_port = cam_obj.get("inputStreamPort")
        out_port = cam_obj.get("outputStreamPort")
        if isinstance(in_port, int) and in_port > 0 and isinstance(out_port, int) and out_port > 0:
            return in_port, out_port

    for cam in _latest_targets["cameras"].values():
        if not isinstance(cam, dict):
            continue
        if int(cam.get("cameraIndex", -1)) == int(cam_idx):
            in_port = cam.get("inputStreamPort")
            out_port = cam.get("outputStreamPort")
            if isinstance(in_port, int) and in_port > 0 and isinstance(out_port, int) and out_port > 0:
                return in_port, out_port

    return default_input, default_output


def _camera_name_for_index(cam_idx):
    """Resolve camera name from cached state by index."""
    target = int(cam_idx)
    for name, cam in _latest_targets["cameras"].items():
        if isinstance(cam, dict) and int(cam.get("cameraIndex", -1)) == target:
            return name
    return None


def _pose_from_transform(transform):
    """Normalize PhotonVision transform payloads into {x,y,z,rx,ry,rz}.

    Supports both nested shape:
      {translation:{x,y,z}, rotation:{x,y,z}}
    and flat shape:
      {x,y,z,angle_x,angle_y,angle_z}
    """
    if not isinstance(transform, dict):
        return None

    translation = transform.get("translation") if isinstance(transform.get("translation"), dict) else transform
    rotation = transform.get("rotation") if isinstance(transform.get("rotation"), dict) else {}

    return {
        "x": round(float(translation.get("x", 0)), 4),
        "y": round(float(translation.get("y", 0)), 4),
        "z": round(float(translation.get("z", 0)), 4),
        "rx": round(float(rotation.get("angle_x", rotation.get("x", transform.get("angle_x", 0)))), 2),
        "ry": round(float(rotation.get("angle_y", rotation.get("y", transform.get("angle_y", 0)))), 2),
        "rz": round(float(rotation.get("angle_z", rotation.get("z", transform.get("angle_z", 0)))), 2),
    }


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


# WebSocket command queue for sending to PV engine
_ws_command_queue = []
_ws_command_lock = threading.Lock()
_ws_connected = False


def _ws_send_command(command_dict):
    """Queue a MessagePack-encoded command to send to PV via WebSocket."""
    with _ws_command_lock:
        _ws_command_queue.append(command_dict)


def _start_target_poller():
    """Start background thread to collect target/pose data.

    Data source priority:
    1. PV WebSocket (ws://engine:5800/websocket_data) - live MessagePack data
    2. ntcore (NetworkTables) - live target data via NT protocol
    3. HTTP API fallback - static config only (no live targets)
    """
    t = threading.Thread(target=_target_poll_loop, daemon=True)
    t.start()


def _read_http_response(sock, max_header_bytes=65536):
    """Read an HTTP response from a socket and return (status, headers, body_start)."""
    header_buf = b""
    while b"\r\n\r\n" not in header_buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Connection closed reading headers")
        header_buf += chunk
        if len(header_buf) > max_header_bytes:
            raise ConnectionError("Header too large")

    header_end = header_buf.index(b"\r\n\r\n") + 4
    headers_raw = header_buf[:header_end]
    body_start = header_buf[header_end:]

    status_line = headers_raw.split(b"\r\n", 1)[0]
    status_parts = status_line.split(b" ", 2)
    if len(status_parts) < 2:
        raise ConnectionError("Malformed HTTP status line")
    status_code = int(status_parts[1])

    headers = {}
    for line in headers_raw.split(b"\r\n")[1:]:
        if not line or b":" not in line:
            continue
        key, value = line.split(b":", 1)
        headers[key.strip().lower()] = value.strip()

    return status_code, headers, body_start


def _extract_first_jpeg(data, boundary=None):
    """Extract the first JPEG payload from bytes that may contain MJPEG multipart data."""
    # Fast path: contiguous JPEG markers.
    start = data.find(b"\xff\xd8")
    if start >= 0:
        end = data.find(b"\xff\xd9", start + 2)
        if end > start:
            return data[start:end + 2]

    if boundary:
        # Boundary can be quoted or prefixed with -- in content-type.
        b = boundary.strip().strip('"')
        if b.startswith("--"):
            b = b[2:]
        marker = ("--" + b).encode("ascii", errors="ignore")
        parts = data.split(marker)
        for part in parts:
            hdr_end = part.find(b"\r\n\r\n")
            if hdr_end < 0:
                continue
            payload = part[hdr_end + 4:]
            s = payload.find(b"\xff\xd8")
            e = payload.find(b"\xff\xd9", s + 2) if s >= 0 else -1
            if s >= 0 and e > s:
                return payload[s:e + 2]

    return None


def _fetch_camera_frame(cam_idx, stream_type="output", timeout=2.5):
    """Fetch a single JPEG frame from PhotonVision stream sources.

    Returns JPEG bytes or None.
    """
    dedicated_port = STREAM_BASE_PORT + (cam_idx * 2) + (1 if stream_type == "output" else 0)
    candidates = [
        (ENGINE_HOST, dedicated_port, "/stream.mjpg"),
        (ENGINE_HOST, dedicated_port, "/stream"),
        (ENGINE_HOST, dedicated_port, "/?action=stream"),
        # Some PV builds may expose streams from the main HTTP server.
        (ENGINE_HOST, ENGINE_PORT, f"/api/camera/{cam_idx}/{stream_type}.mjpg"),
        (ENGINE_HOST, ENGINE_PORT, f"/api/stream/{cam_idx}/{stream_type}"),
        (ENGINE_HOST, ENGINE_PORT, f"/stream/{cam_idx}/{stream_type}"),
    ]

    for host, port, path in candidates:
        sock = None
        try:
            sock = socket.create_connection((host, port), timeout=timeout)
            sock.settimeout(timeout)
            req = (
                f"GET {path} HTTP/1.1\r\n"
                f"Host: {host}:{port}\r\n"
                f"User-Agent: AprilVision-Bridge/3.2\r\n"
                f"Accept: multipart/x-mixed-replace,image/jpeg,*/*\r\n"
                f"Connection: close\r\n\r\n"
            )
            sock.sendall(req.encode("ascii", errors="ignore"))

            status, headers, body = _read_http_response(sock)
            if status != 200:
                continue

            ctype = headers.get(b"content-type", b"").decode("ascii", errors="ignore").lower()
            boundary = None
            if "boundary=" in ctype:
                boundary = ctype.split("boundary=", 1)[1].split(";", 1)[0].strip()

            # Read enough bytes to capture at least one JPEG frame.
            buf = io.BytesIO()
            if body:
                buf.write(body)

            deadline = time.time() + timeout
            while time.time() < deadline and buf.tell() < 2_000_000:
                frame = _extract_first_jpeg(buf.getvalue(), boundary=boundary)
                if frame:
                    return frame
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    break
                if not chunk:
                    break
                buf.write(chunk)

            frame = _extract_first_jpeg(buf.getvalue(), boundary=boundary)
            if frame:
                return frame
        except Exception:
            pass
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
    return None


def _extract_ws_payloads(payload):
    """Yield dict payload candidates from PV websocket frames.

    PV versions may emit:
      - direct dict payloads
      - envelope payloads containing data/message/payload
      - list batches of any of the above
    """
    if isinstance(payload, list):
        for item in payload:
            for d in _extract_ws_payloads(item):
                yield d
        return

    if not isinstance(payload, dict):
        return

    yield payload

    for wrapped_key in ("data", "payload", "message", "results"):
        wrapped = payload.get(wrapped_key)
        if isinstance(wrapped, dict):
            yield wrapped
        elif isinstance(wrapped, list):
            for item in wrapped:
                if isinstance(item, dict):
                    yield item


def _target_poll_loop():
    """Continuously try all data sources in priority order, retrying forever.

    Never gives up on WebSocket - if PV restarts, we reconnect.
    Runs NT and API pollers in parallel as fallback while WebSocket retries.
    """
    global _ws_connected

    # Start fallback pollers in separate threads (they provide data while WS connects)
    fallback_started = False

    def _start_fallback():
        nonlocal fallback_started
        if fallback_started:
            return
        fallback_started = True
        # NT poller thread
        nt_thread = threading.Thread(target=_try_nt_fallback, daemon=True)
        nt_thread.start()

    def _try_nt_fallback():
        nt_client = _try_ntcore()
        if nt_client:
            print("[targets] NT fallback active")
            _poll_ntcore(nt_client)
        else:
            print("[targets] NT unavailable, using HTTP API fallback")
            _poll_api_targets()

    # Main loop: always try WebSocket, restart on disconnect
    while True:
        if _HAS_WEBSOCKET and _HAS_MSGPACK:
            try:
                print(f"[targets] Connecting to PV WebSocket ws://{ENGINE_HOST}:{ENGINE_PORT}/websocket_data ...")
                _poll_pv_websocket()
            except Exception as e:
                print(f"[targets] WebSocket error: {e}")
            _ws_connected = False
            with _targets_lock:
                if _latest_targets["source"] == "websocket":
                    _latest_targets["source"] = "websocket (reconnecting...)"

            # Start fallback on first disconnect
            _start_fallback()
            time.sleep(3)
        else:
            missing = []
            if not _HAS_WEBSOCKET:
                missing.append("websocket-client")
            if not _HAS_MSGPACK:
                missing.append("msgpack")
            print(f"[targets] WebSocket libs missing (pip3 install {' '.join(missing)})")
            _start_fallback()
            break  # No point looping without the libs


def _poll_pv_websocket():
    """Connect to PV's WebSocket and receive live data via MessagePack.

    PhotonVision's WebSocket at /websocket_data sends MessagePack-encoded
    maps containing camera configurations, pipeline results, and system state.

    Inbound (from PV): camera data, FPS, latency, settings updates
    Outbound (to PV): pipeline changes, driver mode, etc.
    """
    global _ws_connected

    ws_url = f"ws://{ENGINE_HOST}:{ENGINE_PORT}/websocket_data"

    # Retry loop - PV might not be ready yet at bridge startup
    while True:
        try:
            ws = _ws_lib.create_connection(ws_url, timeout=10)
            _ws_connected = True
            with _targets_lock:
                _latest_targets["source"] = "websocket"
            print(f"[targets] Connected to PV WebSocket at {ws_url}")

            while True:
                # Send any queued commands
                with _ws_command_lock:
                    while _ws_command_queue:
                        cmd = _ws_command_queue.pop(0)
                        try:
                            packed = msgpack.packb(cmd, use_bin_type=True)
                            ws.send_binary(packed)
                        except Exception as e:
                            print(f"[targets] WS send error: {e}")

                # Receive data
                try:
                    ws.settimeout(0.5)
                    raw = ws.recv()
                except _ws_lib.WebSocketTimeoutException:
                    continue
                except _ws_lib.WebSocketConnectionClosedException:
                    print("[targets] PV WebSocket closed, reconnecting...")
                    break

                if raw is None:
                    continue

                try:
                    if isinstance(raw, bytes):
                        data = msgpack.unpackb(raw, raw=False)
                    else:
                        # Sometimes PV sends text frames
                        try:
                            data = json.loads(raw)
                        except (json.JSONDecodeError, TypeError):
                            continue

                    _process_pv_ws_data(data)
                except Exception as e:
                    # Silently skip malformed frames
                    pass

        except (ConnectionRefusedError, OSError, _ws_lib.WebSocketException) as e:
            _ws_connected = False
            with _targets_lock:
                _latest_targets["source"] = "websocket (connecting...)"
            time.sleep(3)  # Wait for PV to start
        except Exception as e:
            _ws_connected = False
            print(f"[targets] WebSocket error: {e}")
            time.sleep(5)


def _process_pv_ws_data(data):
    """Process a decoded MessagePack frame from PV's WebSocket.

    PV sends Map<String, Object> with various keys depending on the event.
    Known keys from PV source (DataSocketHandler / UIDataPublisher):
      - Camera configuration data (cameraSettings, cameras, etc.)
      - Pipeline results
      - FPS, latency metrics
      - General settings
    """
    with _targets_lock:
        had_payload = False
        for data_item in _extract_ws_payloads(data):
            had_payload = True
            _latest_targets["last_update"] = time.time()
            _latest_targets["nt_connected"] = True  # WS is our "connection"

            # Camera configurations (full state dump on connect)
            cam_data = (data_item.get("cameras") or data_item.get("cameraSettings")
                        or data_item.get("cameraConfigurations") or data_item.get("configuredCameras"))

            if cam_data:
                if isinstance(cam_data, dict):
                    for cam_name, cam_conf in cam_data.items():
                        _update_camera_from_ws(cam_name, cam_conf)
                elif isinstance(cam_data, list):
                    for i, cam_conf in enumerate(cam_data):
                        cam_name = (cam_conf.get("nickname") or cam_conf.get("uniqueName")
                                    or cam_conf.get("name") or f"Camera_{i}")
                        _update_camera_from_ws(cam_name, cam_conf)

            # Per-camera result updates (incremental)
            for key in ("cameraResult", "result", "pipelineResult", "cameraResults", "pipelineResults", "results", "updatePipelineResult"):
                result = data_item.get(key)
                if not result:
                    continue
                if isinstance(result, dict):
                    if any(k in result for k in ("targets", "trackedTargets", "hasTarget", "bestFiducialId", "fiducialId", "fiducialID")):
                        cam_name = (result.get("cameraName") or result.get("nickname")
                                    or result.get("camera") or f"Camera_{result.get('cameraIndex', 0)}")
                        _update_result_from_ws(cam_name, result)
                    else:
                        for cam_name, cam_result in result.items():
                            if isinstance(cam_result, dict):
                                cam_idx = cam_result.get("cameraIndex", cam_name)
                                resolved_name = _camera_name_for_index(cam_idx)
                                _update_result_from_ws(resolved_name or str(cam_name), cam_result)
                elif isinstance(result, list):
                    for i, cam_result in enumerate(result):
                        if isinstance(cam_result, dict):
                            cam_name = (cam_result.get("cameraName") or cam_result.get("nickname")
                                        or cam_result.get("camera") or f"Camera_{cam_result.get('cameraIndex', i)}")
                            _update_result_from_ws(cam_name, cam_result)

            metrics = data_item.get("metrics") if isinstance(data_item.get("metrics"), dict) else {}
            fps = data_item.get("fps", metrics.get("fps"))
            latency = (data_item.get("latency") or data_item.get("latencyMillis") or
                       metrics.get("latency") or metrics.get("latencyMillis"))
            if fps is not None or latency is not None:
                cam_name = data_item.get("cameraName") or data_item.get("camera")
                if cam_name and cam_name in _latest_targets["cameras"]:
                    if fps is not None:
                        _latest_targets["cameras"][cam_name]["fps"] = round(float(fps), 1)
                    if latency is not None:
                        _latest_targets["cameras"][cam_name]["latencyMs"] = round(float(latency), 1)
                else:
                    for cam in _latest_targets["cameras"].values():
                        if fps is not None:
                            cam["fps"] = round(float(fps), 1)
                        if latency is not None:
                            cam["latencyMs"] = round(float(latency), 1)

            general = data_item.get("generalSettings") or data_item.get("general")
            if general and isinstance(general, dict):
                _latest_targets.setdefault("_pv_settings", {}).update(general)

            if "currentCamera" in data_item:
                _latest_targets["_currentCamera"] = data_item["currentCamera"]
            if "currentPipeline" in data_item:
                _latest_targets["_currentPipeline"] = data_item["currentPipeline"]

        if not had_payload:
            return


def _update_camera_from_ws(cam_name, cam_conf):
    """Update camera state from a WS configuration message."""
    if not isinstance(cam_conf, dict):
        return
    existing = _latest_targets["cameras"].get(cam_name, {})

    # Pipeline info
    pipelines = cam_conf.get("pipelineSettings") or cam_conf.get("pipelines") or []
    current_idx = cam_conf.get("currentPipelineIndex") or cam_conf.get("pipelineIndex") or 0
    pipe_name = ""
    pipe_type = ""
    if isinstance(pipelines, list) and pipelines:
        idx = min(current_idx, len(pipelines) - 1)
        pipe = pipelines[idx] if isinstance(pipelines[idx], dict) else {}
        pipe_name = pipe.get("pipelineNickname") or pipe.get("name") or f"Pipeline {idx}"
        pipe_type = pipe.get("pipelineType") or pipe.get("type") or "unknown"

    # Calibration
    cals = cam_conf.get("calibrations") or cam_conf.get("cameraCalibrations") or []
    calibrated = bool(cals)

    existing.update({
        "cameraIndex": cam_conf.get("cameraIndex", existing.get("cameraIndex", 0)),
        "pipelineName": pipe_name,
        "pipelineType": pipe_type,
        "pipelineIndex": current_idx,
        "pipelineCount": len(pipelines) if isinstance(pipelines, list) else 0,
        "calibrated": calibrated,
        "cameraPath": cam_conf.get("path") or cam_conf.get("cameraPath") or "",
        "inputStreamPort": cam_conf.get("inputStreamPort", existing.get("inputStreamPort")),
        "outputStreamPort": cam_conf.get("outputStreamPort", existing.get("outputStreamPort")),
        "driverMode": cam_conf.get("driverMode", False),
    })
    # Preserve any existing target data (don't overwrite live results with config)
    existing.setdefault("hasTarget", False)
    existing.setdefault("latencyMs", 0)
    existing.setdefault("fps", 0)
    existing.setdefault("allTagIds", [])
    existing.setdefault("tagCount", 0)
    _latest_targets["cameras"][cam_name] = existing


def _update_result_from_ws(cam_name, result):
    """Update camera with live pipeline result from WS."""
    existing = _latest_targets["cameras"].get(cam_name, {})

    targets = result.get("targets") or result.get("trackedTargets") or []
    has_target = len(targets) > 0 if isinstance(targets, list) else bool(result.get("hasTarget"))

    best_target = targets[0] if targets and isinstance(targets, list) else result
    try:
        best_id = int(best_target.get("fiducialId") or best_target.get("fid") or
                      best_target.get("bestFiducialId") or best_target.get("fiducialID") or -1)
    except (TypeError, ValueError):
        best_id = -1

    # All detected tag IDs
    all_ids = []
    if isinstance(targets, list):
        for t in targets:
            try:
                fid = int(t.get("fiducialId") or t.get("fid") or t.get("fiducialID") or -1)
            except (TypeError, ValueError):
                fid = -1
            if fid >= 0:
                all_ids.append(fid)

    # Some PV payloads include IDs outside targets array.
    used_ids = (result.get("fiducialIDsUsed") or result.get("fiducialIdsUsed") or
                result.get("tagIds") or result.get("detectedTagIds"))
    if not used_ids and isinstance(result.get("multitagResult"), dict):
        used_ids = (result["multitagResult"].get("fiducialIDsUsed") or
                    result["multitagResult"].get("fiducialIdsUsed"))
    if isinstance(used_ids, list):
        for fid in used_ids:
            try:
                fid_int = int(fid)
            except (TypeError, ValueError):
                continue
            if fid_int >= 0 and fid_int not in all_ids:
                all_ids.append(fid_int)

    # Best target data
    yaw = float(best_target.get("yaw") or best_target.get("targetYaw") or 0)
    pitch = float(best_target.get("pitch") or best_target.get("targetPitch") or 0)
    area = float(best_target.get("area") or best_target.get("targetArea") or 0)
    skew = float(best_target.get("skew") or best_target.get("targetSkew") or 0)

    # 3D pose (solvePnP result)
    pose_data = None
    best_cam_to_target = best_target.get("bestCameraToTarget") or best_target.get("cameraToTarget")
    if best_cam_to_target and isinstance(best_cam_to_target, dict):
        pose_data = _pose_from_transform(best_cam_to_target)

    # Multi-tag pose estimate
    multitag = result.get("multiTagResult") or result.get("multitagResult")
    multitag_pose = None
    if multitag and isinstance(multitag, dict):
        est = multitag.get("estimatedPose") or multitag.get("best") or multitag.get("bestTransform") or multitag
        if isinstance(est, dict):
            multitag_pose = _pose_from_transform(est)
            # Update robot pose from multi-tag
            if multitag_pose and (multitag_pose["x"] != 0 or multitag_pose["y"] != 0):
                _latest_targets["robot_pose"] = {
                    "x": multitag_pose["x"],
                    "y": multitag_pose["y"],
                    "z": multitag_pose["z"],
                    "rotation": multitag_pose["rz"],
                }

    existing.update({
        "cameraIndex": result.get("cameraIndex", existing.get("cameraIndex", 0)),
        "hasTarget": has_target,
        "targetYaw": round(yaw, 2),
        "targetPitch": round(pitch, 2),
        "targetArea": round(area, 2),
        "targetSkew": round(skew, 2),
        "bestFiducialId": best_id,
        "allTagIds": all_ids if all_ids else ([best_id] if best_id >= 0 else []),
        "tagCount": len(all_ids) if all_ids else (1 if best_id >= 0 else 0),
        "pose": pose_data,
        "multitagPose": multitag_pose,
        "latencyMs": round(float(result.get("latencyMillis") or result.get("latency") or
                                  existing.get("latencyMs", 0)), 1),
        "fps": round(float(result.get("fps") or existing.get("fps", 0)), 1),
    })
    _latest_targets["cameras"][cam_name] = existing


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

                    for cam_idx, cam_name in enumerate(subtables):
                        try:
                            cam_table = pv_table.getSubTable(cam_name)
                            has_target = cam_table.getEntry("hasTarget").getBoolean(False)
                            latency = cam_table.getEntry("latencyMillis").getDouble(0)
                            target_yaw = cam_table.getEntry("targetYaw").getDouble(0)
                            target_pitch = cam_table.getEntry("targetPitch").getDouble(0)
                            target_area = cam_table.getEntry("targetArea").getDouble(0)
                            target_skew = cam_table.getEntry("targetSkew").getDouble(0)
                            best_id = cam_table.getEntry("targetFiducialId").getDouble(-1)

                            # Pipeline FPS (heartbeat counter)
                            fps = 0
                            try:
                                fps = cam_table.getEntry("heartbeat").getDouble(0)
                            except Exception:
                                pass

                            # 3D pose of best target
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

                            # Multi-tag: read ALL detected fiducial IDs
                            # PV publishes targetFiducialIds (plural) as int array
                            all_tag_ids = []
                            try:
                                ids_arr = cam_table.getEntry("targetFiducialIds").getIntegerArray([])
                                all_tag_ids = [int(x) for x in ids_arr if x >= 0]
                            except Exception:
                                pass
                            # Fallback: at least include the best target
                            if not all_tag_ids and int(best_id) >= 0:
                                all_tag_ids = [int(best_id)]

                            # Multi-tag pose estimate (field-space)
                            multitag_pose = None
                            try:
                                mt = cam_table.getEntry("multiTagBestPose")
                                mt_arr = mt.getDoubleArray([])
                                if len(mt_arr) >= 3:
                                    multitag_pose = {
                                        "x": round(mt_arr[0], 4),
                                        "y": round(mt_arr[1], 4),
                                        "z": round(mt_arr[2], 4),
                                        "rx": round(mt_arr[3], 2) if len(mt_arr) > 3 else 0,
                                        "ry": round(mt_arr[4], 2) if len(mt_arr) > 4 else 0,
                                        "rz": round(mt_arr[5], 2) if len(mt_arr) > 5 else 0,
                                    }
                            except Exception:
                                pass

                            _latest_targets["cameras"][cam_name] = {
                                "cameraIndex": cam_idx,
                                "hasTarget": has_target,
                                "latencyMs": round(latency, 1),
                                "targetYaw": round(target_yaw, 2),
                                "targetPitch": round(target_pitch, 2),
                                "targetArea": round(target_area, 2),
                                "targetSkew": round(target_skew, 2),
                                "bestFiducialId": int(best_id),
                                "allTagIds": all_tag_ids,
                                "tagCount": len(all_tag_ids),
                                "pose": pose_data,
                                "multitagPose": multitag_pose,
                                "fps": fps,
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
                                    "cameraIndex": i,
                                    "hasTarget": False,
                                    "latencyMs": 0,
                                    "pipelineType": pipe.get("pipelineType", "unknown"),
                                    "pipelineName": pipe.get("pipelineNickname", pipe.get("name", f"Pipeline {current_pipe_idx}")),
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

        PhotonVision/cscore serves MJPEG as multipart/x-mixed-replace on
        dedicated ports.  We relay the raw TCP bytes so the browser gets
        a live, continuous stream.

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
        input_port, output_port = _stream_ports_for_camera(cam_idx)
        port = output_port if stream_type == 'output' else input_port

        sock = None
        # cscore MJPEG server paths in order of likelihood
        stream_paths = ["/stream.mjpg", "/stream", "/?action=stream", "/"]
        try:
            # Try each path until we get a 200
            connected = False
            headers_raw = b""
            body_start = b""

            for try_path in stream_paths:
                try:
                    sock = socket.create_connection((ENGINE_HOST, port), timeout=5)
                    sock.settimeout(5)

                    # Use HTTP/1.1 to keep the connection alive for streaming
                    request = (
                        f"GET {try_path} HTTP/1.1\r\n"
                        f"Host: {ENGINE_HOST}:{port}\r\n"
                        f"User-Agent: AprilVision-Bridge/3.2\r\n"
                        f"Accept: */*\r\n"
                        f"Connection: keep-alive\r\n"
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

                    # Non-200: close and try next path
                    sock.close()
                    sock = None
                except (socket.timeout, ConnectionError, OSError):
                    if sock:
                        try:
                            sock.close()
                        except Exception:
                            pass
                        sock = None
                    continue

            if not connected or not sock:
                raise ConnectionError(f"No MJPEG stream on port {port}")

            # Forward response headers to the browser
            # Parse the Content-Type from cscore (contains the boundary marker)
            content_type = "multipart/x-mixed-replace; boundary=myboundary"
            for header_line in headers_raw.split(b"\r\n")[1:]:
                if not header_line:
                    continue
                if b":" in header_line:
                    key, value = header_line.split(b":", 1)
                    if key.strip().lower() == b"content-type":
                        content_type = value.strip().decode("ascii", errors="replace")

            # Send our own clean response headers
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            # Do NOT send Connection: close - this is a continuous stream
            self.end_headers()

            # Send any body data that arrived with the headers
            if body_start:
                self.wfile.write(body_start)
                self.wfile.flush()

            # Relay MJPEG stream data continuously
            while True:
                readable, _, _ = select.select([sock], [], [], 30)
                if not readable:
                    break  # 30s with no data = stream dead
                try:
                    data = sock.recv(65536)
                except (socket.timeout, OSError):
                    break
                if not data:
                    break
                self.wfile.write(data)
                self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # Browser closed the tab / navigated away
        except Exception as e:
            try:
                self._send_error(503, "Stream Unavailable",
                                 f"Camera {cam_idx} {stream_type} stream not available on port {port}. "
                                 f"Ensure PhotonVision is running with a camera connected. Error: {e}")
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
        elif endpoint == 'stream-test-suite':
            return self._api_stream_test_suite()
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
        elif endpoint == 'set-pipeline' and method == 'POST':
            return self._api_set_pipeline()
        elif endpoint == 'set-driver-mode' and method == 'POST':
            return self._api_set_driver_mode()
        elif endpoint == 'performance':
            return self._api_performance()
        elif endpoint.startswith('frame/'):
            return self._api_frame(endpoint)
        else:
            self._send_json({"error": "Unknown endpoint"}, 404)

    def _api_frame(self, endpoint):
        """Return a single JPEG frame for a camera stream.

        Endpoint: /api/av/frame/<camera_index>/<input|output>
        Useful fallback when MJPEG streaming is blocked by browser/network.
        """
        try:
            parts = endpoint.split('/')
            if len(parts) < 2:
                return self._send_json({"error": "Use /api/av/frame/<index>/<input|output>"}, 400)

            cam_idx = int(parts[1])
            stream_type = parts[2] if len(parts) > 2 else "output"
            if stream_type not in ("input", "output"):
                stream_type = "output"

            frame = _fetch_camera_frame(cam_idx, stream_type=stream_type, timeout=2.5)
            if not frame:
                return self._send_json({"error": "No frame available", "cameraIndex": cam_idx, "streamType": stream_type}, 503)

            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(frame)
        except Exception as e:
            self._send_json({"error": str(e)}, 500)

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
                    'inputStreamPort': _stream_ports_for_camera(i, cam)[0],
                    'outputStreamPort': _stream_ports_for_camera(i, cam)[1],
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
        with _targets_lock:
            data_source = _latest_targets["source"]
        self._send_json({
            'version': 'AprilVision 3.2',
            'engine_online': online,
            'engine_error': err,
            'engine_api_path': api_path,
            'proxy_online': True,
            'ws_connected': _ws_connected,
            'data_source': data_source,
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
                'inputPort': _stream_ports_for_camera(i, cam)[0],
                'outputPort': _stream_ports_for_camera(i, cam)[1],
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
            in_port, out_port = _stream_ports_for_camera(i)
            results['stream_probes'].append({
                'camera_index': i,
                'input_port': in_port,
                'output_port': out_port,
                'input_status': self._check_stream_port(in_port),
                'output_status': self._check_stream_port(out_port),
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
                input_port, output_port = _stream_ports_for_camera(i, cam)
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
                input_port, output_port = _stream_ports_for_camera(i)
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

    def _api_stream_test_suite(self):
        """Run stream diagnostics for dashboard troubleshooting.

        Verifies per-camera input/output stream reachability and grabs one
        JPEG frame from each stream path as a practical end-to-end test.
        """
        started = time.time()
        online, data, _, err = _engine_probe()
        cam_list = _extract_cameras(data) if data else []

        camera_results = []
        passing_cameras = 0

        for i, cam in enumerate(cam_list):
            in_port, out_port = _stream_ports_for_camera(i, cam)
            input_status = self._check_stream_port(in_port)
            output_status = self._check_stream_port(out_port)

            input_frame = _fetch_camera_frame(i, "input", timeout=2.5)
            output_frame = _fetch_camera_frame(i, "output", timeout=2.5)

            camera_ok = (
                input_status in ("live", "responding") and
                output_status in ("live", "responding") and
                bool(input_frame) and bool(output_frame)
            )
            if camera_ok:
                passing_cameras += 1

            camera_results.append({
                "camera": cam.get('nickname', cam.get('uniqueName', cam.get('name', f'Camera {i}'))),
                "index": i,
                "inputPort": in_port,
                "outputPort": out_port,
                "inputStatus": input_status,
                "outputStatus": output_status,
                "inputFrameOk": bool(input_frame),
                "outputFrameOk": bool(output_frame),
                "inputFrameBytes": len(input_frame) if input_frame else 0,
                "outputFrameBytes": len(output_frame) if output_frame else 0,
                "inputUrl": f"/stream/{i}/input",
                "outputUrl": f"/stream/{i}/output",
                "pass": camera_ok,
            })

        total = len(camera_results)
        suite_pass = online and total > 0 and passing_cameras == total

        self._send_json({
            "startedAt": int(started),
            "runtimeMs": int((time.time() - started) * 1000),
            "engineOnline": online,
            "engineError": err,
            "cameraCount": total,
            "passingCameras": passing_cameras,
            "overallPass": suite_pass,
            "results": camera_results,
            "notes": [
                "Input/output stream status must be reachable and a sample JPEG frame must be fetchable.",
                "If status is unreachable, verify PhotonVision camera assignment and USB connection.",
                "If frame fetch fails but port is reachable, stream may be blocked or still warming up.",
            ],
        })

    def _api_camera_info(self):
        """Rich camera information combining system devices and engine data."""
        system_cameras = get_system_cameras()
        engine_cameras = []
        online, data, api_path, err = _engine_probe()
        cam_list = _extract_cameras(data) if data else []

        for i, cam in enumerate(cam_list):
            input_port, output_port = _stream_ports_for_camera(i, cam)
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
        """Return latest target/pose data from WebSocket, NT, or engine API."""
        with _targets_lock:
            data = {
                "cameras": dict(_latest_targets["cameras"]),
                "robotPose": _latest_targets["robot_pose"],
                "lastUpdate": _latest_targets["last_update"],
                "ntConnected": _latest_targets["nt_connected"],
                "wsConnected": _ws_connected,
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
            _, port = _stream_ports_for_camera(i)  # output stream
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

    def _api_set_pipeline(self):
        """Switch the active pipeline for a camera.

        Uses PV's WebSocket protocol (preferred) or HTTP fallback.
        PV WebSocket command: {"currentPipeline": index, "currentCamera": camIndex}
        """
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(content_length)) if content_length > 0 else {}
            cam_index = body.get("cameraIndex", 0)
            pipe_index = body.get("pipelineIndex", 0)

            # Prefer WebSocket command (same protocol PV's own UI uses)
            if _ws_connected and _HAS_MSGPACK:
                _ws_send_command({"currentCamera": cam_index})
                _ws_send_command({"currentPipeline": pipe_index})
                self._send_json({"ok": True, "cameraIndex": cam_index,
                                 "pipelineIndex": pipe_index, "method": "websocket"})
                return

            # HTTP fallback: try the engine's REST API
            for api_path in ["/api/settings/general", "/api/v1/setPipeline"]:
                try:
                    payload = json.dumps({"cameraIndex": cam_index, "pipelineIndex": pipe_index}).encode()
                    conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=5)
                    conn.request("POST", api_path, body=payload,
                                 headers={"Content-Type": "application/json"})
                    resp = conn.getresponse()
                    resp.read()
                    conn.close()
                    if resp.status < 300:
                        self._send_json({"ok": True, "cameraIndex": cam_index,
                                         "pipelineIndex": pipe_index, "method": "http"})
                        return
                except Exception:
                    continue
            self._send_json({"ok": False, "error": "No API path succeeded"}, 502)
        except Exception as e:
            self._send_json({"ok": False, "error": str(e)}, 500)

    def _api_set_driver_mode(self):
        """Toggle driver mode for a camera (streams only, no processing).

        PV WebSocket command: {"driverMode": true/false}
        """
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(content_length)) if content_length > 0 else {}
            cam_index = body.get("cameraIndex", 0)
            enabled = body.get("enabled", True)

            # Prefer WebSocket command
            if _ws_connected and _HAS_MSGPACK:
                _ws_send_command({"currentCamera": cam_index})
                _ws_send_command({"driverMode": enabled})
                self._send_json({"ok": True, "cameraIndex": cam_index,
                                 "driverMode": enabled, "method": "websocket"})
                return

            # HTTP fallback
            try:
                payload = json.dumps({"cameraIndex": cam_index, "driverMode": enabled}).encode()
                conn = http.client.HTTPConnection(ENGINE_HOST, ENGINE_PORT, timeout=5)
                conn.request("POST", "/api/settings/general", body=payload,
                             headers={"Content-Type": "application/json"})
                resp = conn.getresponse()
                resp.read()
                conn.close()
                self._send_json({"ok": resp.status < 300, "cameraIndex": cam_index,
                                 "driverMode": enabled, "method": "http"})
            except Exception as e:
                self._send_json({"ok": False, "error": str(e)}, 502)
        except Exception as e:
            self._send_json({"ok": False, "error": str(e)}, 500)

    def _api_performance(self):
        """Return performance metrics: FPS, latency, CPU, memory."""
        perf = {"cameras": {}, "system": {}}
        with _targets_lock:
            for cam_name, cam_data in _latest_targets["cameras"].items():
                perf["cameras"][cam_name] = {
                    "fps": cam_data.get("fps", 0),
                    "latencyMs": cam_data.get("latencyMs", 0),
                    "tagCount": cam_data.get("tagCount", 0),
                    "hasTarget": cam_data.get("hasTarget", False),
                }
        # System metrics
        try:
            with open("/proc/loadavg", "r") as f:
                parts = f.read().split()
                perf["system"]["loadAvg1m"] = float(parts[0])
                perf["system"]["loadAvg5m"] = float(parts[1])
        except Exception:
            pass
        try:
            with open("/proc/meminfo", "r") as f:
                mem = {}
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        mem[parts[0].rstrip(':')] = int(parts[1])
                total = mem.get("MemTotal", 1)
                avail = mem.get("MemAvailable", total)
                perf["system"]["memUsedPct"] = round(100 * (1 - avail / total), 1)
                perf["system"]["memTotalMB"] = round(total / 1024)
                perf["system"]["memAvailMB"] = round(avail / 1024)
        except Exception:
            pass
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                perf["system"]["cpuTempC"] = round(int(f.read().strip()) / 1000, 1)
        except Exception:
            pass
        perf["system"]["uptimeS"] = int(time.time() - START_TIME)
        self._send_json(perf)

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
