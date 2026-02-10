/**
 * FRC Vision Dashboard - JavaScript Application
 */

class VisionDashboard {
    constructor() {
        this.eventSource = null;
        this.latestData = {
            detections: {},
            fused: null,
            status: null
        };
        this.showOverlay = true;
        this.fieldCanvas = null;
        this.fieldCtx = null;
        this.streamRetryTimers = {};
        this.healthCheckInterval = null;
        this.NUM_CAMERAS = 3;
        this.serverAlive = false;

        this.init();
    }

    init() {
        // Start camera streams immediately
        this.setupCameraStreams();

        // Setup event source for SSE
        this.connectEventSource();

        // Setup canvas overlays
        this.setupCanvasOverlays();

        // Setup field display
        this.setupFieldDisplay();

        // Setup configuration controls
        this.setupConfigControls();

        // Setup accuracy testing controls
        this.setupAccuracyControls();

        // Load current config
        this.loadConfig();

        // Start health check - detects server restarts and reconnects streams
        this.startHealthCheck();

        console.log('Vision Dashboard initialized');
    }

    // =========================================================================
    // Camera Stream Management
    // =========================================================================

    setupCameraStreams() {
        for (let i = 0; i < this.NUM_CAMERAS; i++) {
            this.startCameraStream(i);
        }
    }

    startCameraStream(camId) {
        const img = document.getElementById(`cam${camId}-img`);
        if (!img) return;

        // Clear any pending retry timer
        if (this.streamRetryTimers[camId]) {
            clearTimeout(this.streamRetryTimers[camId]);
            this.streamRetryTimers[camId] = null;
        }

        // Force-reload by clearing src first, then setting new URL with cache-buster
        img.src = '';
        const streamUrl = `/cam${camId}.mjpeg?t=${Date.now()}`;

        // For MJPEG streams, onload fires once when the first frame arrives.
        // onerror fires if the connection is refused or fails initially.
        // But if the server dies MID-STREAM, neither fires - the image just freezes.
        // That's why we use the health check below to detect server restarts.

        img.onload = () => {
            const panel = img.closest('.camera-panel');
            if (panel) {
                panel.classList.remove('stream-error');
                panel.classList.add('stream-active');
            }
        };

        img.onerror = () => {
            console.warn(`Camera ${camId} stream error, retrying in 2s...`);
            const panel = img.closest('.camera-panel');
            if (panel) {
                panel.classList.add('stream-error');
                panel.classList.remove('stream-active');
            }

            // Retry after 2 seconds
            this.streamRetryTimers[camId] = setTimeout(() => {
                this.startCameraStream(camId);
            }, 2000);
        };

        img.src = streamUrl;
    }

    restartAllStreams() {
        console.log('Restarting all camera streams...');
        for (let i = 0; i < this.NUM_CAMERAS; i++) {
            this.startCameraStream(i);
        }
    }

    // =========================================================================
    // Health Check - Detects server restarts
    // =========================================================================

    startHealthCheck() {
        // Poll /health every 3 seconds. If server was down and comes back,
        // restart all MJPEG streams (they freeze when server dies mid-stream).
        this.healthCheckInterval = setInterval(async () => {
            try {
                const response = await fetch('/health', {
                    signal: AbortSignal.timeout(2000)
                });
                if (response.ok) {
                    if (!this.serverAlive) {
                        // Server just came back - restart everything
                        console.log('Server reconnected, restarting streams...');
                        this.serverAlive = true;
                        this.restartAllStreams();
                        this.loadConfig();
                    }
                } else {
                    this.serverAlive = false;
                }
            } catch {
                if (this.serverAlive) {
                    console.warn('Server connection lost');
                    this.serverAlive = false;
                    // Mark all streams as errored
                    for (let i = 0; i < this.NUM_CAMERAS; i++) {
                        const panel = document.querySelector(`#cam${i}-img`)?.closest('.camera-panel');
                        if (panel) {
                            panel.classList.add('stream-error');
                            panel.classList.remove('stream-active');
                        }
                    }
                }
            }
        }, 3000);

        // Initial state: try to connect immediately
        this.serverAlive = true;
    }

    // =========================================================================
    // SSE Event Source
    // =========================================================================

    connectEventSource() {
        // Close existing connection first
        if (this.eventSource) {
            this.eventSource.close();
            this.eventSource = null;
        }

        this.eventSource = new EventSource('/events');

        this.eventSource.addEventListener('detections', (e) => {
            const data = JSON.parse(e.data);
            this.handleDetections(data);
        });

        this.eventSource.addEventListener('fused', (e) => {
            const data = JSON.parse(e.data);
            this.handleFusedPose(data);
        });

        this.eventSource.addEventListener('status', (e) => {
            const data = JSON.parse(e.data);
            this.handleStatus(data);
        });

        this.eventSource.onerror = () => {
            console.log('SSE disconnected, reconnecting in 2s...');
            if (this.eventSource) {
                this.eventSource.close();
                this.eventSource = null;
            }
            setTimeout(() => this.connectEventSource(), 2000);
        };

        this.eventSource.onopen = () => {
            console.log('SSE connected');
            // SSE reconnected means server is (re)started - restart streams
            if (this.serverAlive) {
                this.restartAllStreams();
            }
            this.serverAlive = true;
        };
    }

    // =========================================================================
    // Canvas Overlays
    // =========================================================================

    setupCanvasOverlays() {
        for (let i = 0; i < this.NUM_CAMERAS; i++) {
            const img = document.getElementById(`cam${i}-img`);
            const canvas = document.getElementById(`cam${i}-overlay`);

            if (img && canvas) {
                const resizeCanvas = () => {
                    canvas.width = img.offsetWidth;
                    canvas.height = img.offsetHeight;
                };

                // Use ResizeObserver instead of onload (which conflicts with stream handler)
                const observer = new ResizeObserver(resizeCanvas);
                observer.observe(img);
                window.addEventListener('resize', resizeCanvas);
                resizeCanvas();
            }
        }
    }

    setupFieldDisplay() {
        this.fieldCanvas = document.getElementById('field-canvas');
        if (this.fieldCanvas) {
            this.fieldCtx = this.fieldCanvas.getContext('2d');
            this.drawField();
        }
    }

    setupConfigControls() {
        // Range input value displays
        const rangeInputs = ['decimation', 'min-margin', 'max-tags', 'filter-alpha', 'jpeg-quality'];
        rangeInputs.forEach(id => {
            const input = document.getElementById(id);
            const display = document.getElementById(`${id}-value`);
            if (input && display) {
                input.addEventListener('input', () => {
                    display.textContent = input.value;
                });
            }
        });

        // Show overlay toggle
        const overlayToggle = document.getElementById('show-overlay');
        if (overlayToggle) {
            overlayToggle.addEventListener('change', (e) => {
                this.showOverlay = e.target.checked;
                this.clearAllOverlays();
            });
        }

        // Apply config button
        const applyBtn = document.getElementById('apply-config');
        if (applyBtn) {
            applyBtn.addEventListener('click', () => this.applyConfig());
        }

        // Reload config button
        const reloadBtn = document.getElementById('reload-config');
        if (reloadBtn) {
            reloadBtn.addEventListener('click', () => this.reloadConfig());
        }
    }

    // =========================================================================
    // Data Handlers
    // =========================================================================

    handleDetections(data) {
        const camId = data.camera_id;
        this.latestData.detections[camId] = data;

        // Update camera stats
        const fpsEl = document.querySelector(`.fps[data-cam="${camId}"]`);
        const latencyEl = document.querySelector(`.latency[data-cam="${camId}"]`);
        const tagsEl = document.querySelector(`.tags[data-cam="${camId}"]`);

        if (fpsEl) {
            const fps = (1000 / Math.max(1, data.latency_ms)).toFixed(1);
            fpsEl.textContent = `${fps} fps`;
        }
        if (latencyEl) {
            latencyEl.textContent = `${data.latency_ms.toFixed(1)} ms`;
        }
        if (tagsEl) {
            tagsEl.textContent = `${data.tags.length} tags`;
        }

        // Update debug per-tag data
        this.updateDebugTags(data);

        // Update debug latency
        this.updateDebugLatency(camId, data);

        // Draw overlay
        if (this.showOverlay) {
            this.drawCameraOverlay(camId, data);
        }
    }

    handleFusedPose(data) {
        this.latestData.fused = data;

        // Update pose display
        const pose = data.pose_filtered;
        document.getElementById('pose-x').textContent = pose.x.toFixed(2);
        document.getElementById('pose-y').textContent = pose.y.toFixed(2);
        document.getElementById('pose-theta').textContent = pose.theta_deg.toFixed(1);

        // Quality
        const quality = data.quality;
        document.getElementById('confidence-value').textContent =
            `${(quality.confidence * 100).toFixed(0)}%`;
        document.getElementById('confidence-fill').style.width =
            `${quality.confidence * 100}%`;

        document.getElementById('total-tags').textContent = data.total_tags;
        document.getElementById('cameras-count').textContent = data.cameras_contributing;

        // Std devs
        document.getElementById('std-x').textContent = quality.std_dev_x.toFixed(3);
        document.getElementById('std-y').textContent = quality.std_dev_y.toFixed(3);
        document.getElementById('std-theta').textContent = quality.std_dev_theta.toFixed(3);

        // Draw field
        if (data.valid) {
            this.drawField(pose, data.reference);
        }

        // Update accuracy error display
        if (data.reference && data.reference.set) {
            const el = document.getElementById('accuracy-result');
            if (el) el.style.display = 'block';
            const setVal = (id, val, digits=3) => {
                const e = document.getElementById(id);
                if (e) e.textContent = val.toFixed(digits);
            };
            setVal('err-x', data.reference.error_x);
            setVal('err-y', data.reference.error_y);
            setVal('err-theta', data.reference.error_theta_deg, 1);
            setVal('err-dist', data.reference.error_distance);
        }
    }

    handleStatus(data) {
        this.latestData.status = data;

        // Uptime
        const hours = Math.floor(data.uptime / 3600);
        const mins = Math.floor((data.uptime % 3600) / 60);
        const secs = Math.floor(data.uptime % 60);
        document.getElementById('status-uptime').textContent =
            `Uptime: ${hours}h ${mins}m ${secs}s`;

        // CPU temp
        document.getElementById('status-temp').textContent =
            `CPU: ${data.cpu_temp.toFixed(1)}°C`;

        // SSE clients
        document.getElementById('status-clients').textContent =
            `Clients: ${data.sse_clients}`;

        // NT status (from actual connection status - Phase 3 fix)
        const ntStatus = document.getElementById('status-nt');
        const ntStatusText = document.getElementById('nt-status-text');
        const ntServerIp = document.getElementById('nt-server-ip');

        // Use nt_connected from status API
        const isNTConnected = data.nt_connected || false;
        const ntConnCount = data.nt_connection_count || 0;
        const serverIp = data.nt_server_ip || '';

        if (isNTConnected && ntConnCount > 0) {
            ntStatusText.textContent = 'Connected';
            ntStatus.classList.remove('disconnected');
            ntStatus.classList.add('connected');
            if (serverIp && serverIp !== 'disconnected') {
                ntServerIp.textContent = `(${serverIp})`;
            } else {
                ntServerIp.textContent = '';
            }
        } else {
            ntStatusText.textContent = 'Disconnected';
            ntStatus.classList.remove('connected');
            ntStatus.classList.add('disconnected');
            ntServerIp.textContent = '';
        }
    }

    // =========================================================================
    // Drawing
    // =========================================================================

    // =========================================================================
    // Debug Data
    // =========================================================================

    updateDebugTags(data) {
        const el = document.getElementById('debug-tags');
        if (!el) return;

        if (data.tags.length === 0) {
            el.innerHTML = `<span class="muted">Cam ${data.camera_id}: No tags</span>`;
            return;
        }

        let html = `<div class="debug-cam-header">Cam ${data.camera_id}</div>`;
        for (const tag of data.tags) {
            const dist = tag.distance_m ? tag.distance_m.toFixed(2) + 'm' : '--';
            const reproj = tag.reproj_error ? tag.reproj_error.toFixed(1) + 'px' : '--';
            const pxSize = tag.pixel_size ? tag.pixel_size.toFixed(0) + 'px' : '--';
            const ambig = (tag.ambiguity !== undefined) ? tag.ambiguity.toFixed(2) : '--';
            const ambigClass = (tag.ambiguity < 0.2) ? 'good' : (tag.ambiguity < 0.5) ? 'warn' : 'bad';

            html += `<div class="debug-tag-row">
                <span class="tag-id">Tag ${tag.id}</span>
                <span class="tag-dist">${dist}</span>
                <span class="tag-px">${pxSize}</span>
                <span class="tag-reproj">${reproj}</span>
                <span class="tag-ambig ${ambigClass}">A:${ambig}</span>
                <span class="tag-margin">M:${tag.margin.toFixed(0)}</span>
            </div>`;
        }

        if (data.robot_pose) {
            html += `<div class="debug-pose-row">Pose: (${data.robot_pose.x.toFixed(2)}, ${data.robot_pose.y.toFixed(2)}, ${data.robot_pose.theta_deg.toFixed(1)}°)</div>`;
        }

        el.innerHTML = html;
    }

    updateDebugLatency(camId, data) {
        const el = document.getElementById('debug-latency');
        if (!el) return;

        // Store per-camera latency
        if (!this._latencyData) this._latencyData = {};
        this._latencyData[camId] = data.latency || {
            detect_ms: 0, pose_ms: 0, total_ms: data.latency_ms
        };

        let html = '';
        for (const [cam, lat] of Object.entries(this._latencyData)) {
            html += `<div class="debug-latency-row">
                <span>Cam ${cam}:</span>
                <span>Det: ${lat.detect_ms.toFixed(1)}ms</span>
                <span>Pose: ${lat.pose_ms.toFixed(1)}ms</span>
                <span>Total: ${lat.total_ms.toFixed(1)}ms</span>
            </div>`;
        }
        el.innerHTML = html;
    }

    setupAccuracyControls() {
        const setBtn = document.getElementById('set-reference');
        const clearBtn = document.getElementById('clear-reference');

        if (setBtn) {
            setBtn.addEventListener('click', async () => {
                const x = parseFloat(document.getElementById('ref-x').value) || 0;
                const y = parseFloat(document.getElementById('ref-y').value) || 0;
                const theta_deg = parseFloat(document.getElementById('ref-theta').value) || 0;

                try {
                    await fetch('/api/debug/reference', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({x, y, theta_deg})
                    });
                    document.getElementById('accuracy-result').style.display = 'block';
                } catch (e) {
                    console.error('Failed to set reference:', e);
                }
            });
        }

        if (clearBtn) {
            clearBtn.addEventListener('click', async () => {
                try {
                    await fetch('/api/debug/reference', {method: 'DELETE'});
                    document.getElementById('accuracy-result').style.display = 'none';
                } catch (e) {
                    console.error('Failed to clear reference:', e);
                }
            });
        }
    }

    // =========================================================================
    // Drawing
    // =========================================================================

    drawCameraOverlay(camId, data) {
        const canvas = document.getElementById(`cam${camId}-overlay`);
        const img = document.getElementById(`cam${camId}-img`);
        if (!canvas || !img) return;

        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Calculate scale factors
        const scaleX = canvas.width / (img.naturalWidth || 640);
        const scaleY = canvas.height / (img.naturalHeight || 480);

        // Draw each tag
        data.tags.forEach(tag => {
            const corners = tag.corners.map(c => ({
                x: c[0] * scaleX,
                y: c[1] * scaleY
            }));

            // Draw corners and edges
            ctx.strokeStyle = '#00ff88';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(corners[0].x, corners[0].y);
            for (let i = 1; i <= 4; i++) {
                ctx.lineTo(corners[i % 4].x, corners[i % 4].y);
            }
            ctx.stroke();

            // Draw corner dots
            ctx.fillStyle = '#00ff88';
            corners.forEach(c => {
                ctx.beginPath();
                ctx.arc(c.x, c.y, 3, 0, Math.PI * 2);
                ctx.fill();
            });

            // Draw ID
            const center = tag.center;
            ctx.fillStyle = '#e94560';
            ctx.font = 'bold 14px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText(tag.id.toString(), center[0] * scaleX, center[1] * scaleY - 10);

            // Draw margin and distance
            ctx.fillStyle = '#ffffff';
            ctx.font = '10px sans-serif';
            const marginStr = `M:${tag.margin.toFixed(0)}`;
            const distStr = tag.distance_m ? `${tag.distance_m.toFixed(2)}m` : '';
            ctx.fillText(`${marginStr} ${distStr}`, center[0] * scaleX, center[1] * scaleY + 20);
        });
    }

    clearAllOverlays() {
        for (let i = 0; i < this.NUM_CAMERAS; i++) {
            const canvas = document.getElementById(`cam${i}-overlay`);
            if (canvas) {
                const ctx = canvas.getContext('2d');
                ctx.clearRect(0, 0, canvas.width, canvas.height);
            }
        }
    }

    drawField(pose = null, reference = null) {
        if (!this.fieldCtx) return;

        const ctx = this.fieldCtx;
        const width = this.fieldCanvas.width;
        const height = this.fieldCanvas.height;

        // FRC 2026 REBUILT field dimensions (meters, welded variant)
        const FIELD_LENGTH = 16.541;
        const FIELD_WIDTH = 8.069;

        // Scale
        const scale = Math.min(width / FIELD_LENGTH, height / FIELD_WIDTH);
        const offsetX = (width - FIELD_LENGTH * scale) / 2;
        const offsetY = (height - FIELD_WIDTH * scale) / 2;

        // Clear
        ctx.fillStyle = '#0a0a15';
        ctx.fillRect(0, 0, width, height);

        // Field outline
        ctx.strokeStyle = '#2a3a5e';
        ctx.lineWidth = 2;
        ctx.strokeRect(offsetX, offsetY, FIELD_LENGTH * scale, FIELD_WIDTH * scale);

        // Center line
        ctx.strokeStyle = '#1a2a4e';
        ctx.beginPath();
        ctx.moveTo(offsetX + FIELD_LENGTH * scale / 2, offsetY);
        ctx.lineTo(offsetX + FIELD_LENGTH * scale / 2, offsetY + FIELD_WIDTH * scale);
        ctx.stroke();

        // Alliance zones
        ctx.fillStyle = 'rgba(255, 0, 0, 0.1)';
        ctx.fillRect(offsetX + FIELD_LENGTH * scale * 0.8, offsetY, FIELD_LENGTH * scale * 0.2, FIELD_WIDTH * scale);
        ctx.fillStyle = 'rgba(0, 0, 255, 0.1)';
        ctx.fillRect(offsetX, offsetY, FIELD_LENGTH * scale * 0.2, FIELD_WIDTH * scale);

        // Draw robot pose
        if (pose) {
            const robotX = offsetX + pose.x * scale;
            const robotY = offsetY + (FIELD_WIDTH - pose.y) * scale;
            const robotSize = 0.5 * scale;

            // Robot body
            ctx.save();
            ctx.translate(robotX, robotY);
            ctx.rotate(-pose.theta);

            ctx.fillStyle = '#00ff88';
            ctx.fillRect(-robotSize / 2, -robotSize / 2, robotSize, robotSize);

            // Direction arrow
            ctx.fillStyle = '#e94560';
            ctx.beginPath();
            ctx.moveTo(robotSize / 2 + 5, 0);
            ctx.lineTo(robotSize / 2 - 5, -8);
            ctx.lineTo(robotSize / 2 - 5, 8);
            ctx.closePath();
            ctx.fill();

            ctx.restore();

            // Position label
            ctx.fillStyle = '#ffffff';
            ctx.font = '10px sans-serif';
            ctx.textAlign = 'left';
            ctx.fillText(
                `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.theta_deg.toFixed(1)}°)`,
                offsetX + 5, offsetY + 12
            );
        }

        // Draw reference position (yellow outline) if set
        if (reference && reference.set) {
            const refX = offsetX + reference.ref_x * scale;
            const refY = offsetY + (FIELD_WIDTH - reference.ref_y) * scale;
            const robotSize = 0.5 * scale;

            ctx.save();
            ctx.translate(refX, refY);
            ctx.rotate(-reference.ref_theta || 0);

            ctx.strokeStyle = '#ffaa00';
            ctx.lineWidth = 2;
            ctx.setLineDash([4, 4]);
            ctx.strokeRect(-robotSize / 2, -robotSize / 2, robotSize, robotSize);
            ctx.setLineDash([]);

            ctx.restore();

            // Show error distance
            if (reference.error_distance !== undefined) {
                ctx.fillStyle = '#ffaa00';
                ctx.font = '10px sans-serif';
                ctx.textAlign = 'right';
                ctx.fillText(
                    `Err: ${reference.error_distance.toFixed(3)}m`,
                    offsetX + FIELD_LENGTH * scale - 5, offsetY + 12
                );
            }
        }
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    async loadConfig() {
        try {
            const response = await fetch('/api/config');
            const config = await response.json();

            // Update UI with config values
            if (config.detector) {
                this.setInputValue('decimation', config.detector.decimation);
                this.setInputValue('min-margin', config.detector.min_margin);
                this.setInputValue('max-tags', config.detector.max_tags_per_frame);
            }
            if (config.tracker) {
                document.getElementById('tracking-enabled').checked = config.tracker.enable;
                this.setInputValue('filter-alpha', config.tracker.filter_alpha);
            }
            if (config.performance) {
                this.setInputValue('jpeg-quality', config.performance.jpeg_quality);
            }
        } catch (e) {
            console.error('Failed to load config:', e);
        }
    }

    setInputValue(id, value) {
        const input = document.getElementById(id);
        const display = document.getElementById(`${id}-value`);
        if (input) {
            input.value = value;
            if (display) display.textContent = value;
        }
    }

    async applyConfig() {
        const config = {
            detector: {
                decimation: parseInt(document.getElementById('decimation').value),
                min_margin: parseInt(document.getElementById('min-margin').value),
                max_tags_per_frame: parseInt(document.getElementById('max-tags').value)
            },
            tracker: {
                enable: document.getElementById('tracking-enabled').checked,
                filter_alpha: parseFloat(document.getElementById('filter-alpha').value)
            },
            performance: {
                jpeg_quality: parseInt(document.getElementById('jpeg-quality').value)
            }
        };

        try {
            const response = await fetch('/api/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });
            const result = await response.json();
            console.log('Config applied:', result);
        } catch (e) {
            console.error('Failed to apply config:', e);
            alert('Failed to apply configuration');
        }
    }

    async reloadConfig() {
        try {
            const response = await fetch('/api/config/reload', { method: 'POST' });
            const result = await response.json();
            console.log('Config reloaded:', result);
            this.loadConfig();
        } catch (e) {
            console.error('Failed to reload config:', e);
            alert('Failed to reload configuration');
        }
    }
}

// Initialize dashboard when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.dashboard = new VisionDashboard();
});

// =========================================================================
// Phase 3: Testing Modes & Auto-Align
// =========================================================================

// Testing mode controls
document.getElementById('start-calibration-mode')?.addEventListener('click', async () => {
    try {
        const response = await fetch('/api/mode/calibration', { method: 'POST' });
        if (response.ok) {
            showTestMode('Calibration Mode', 'Point camera at ChArUco board from various angles. Press SPACE to capture frames. Press C to compute calibration. Press Q to quit.');
        } else {
            alert('Failed to start calibration mode');
        }
    } catch (err) {
        console.error('Error starting calibration:', err);
        alert('Error starting calibration mode');
    }
});

document.getElementById('start-validation-mode')?.addEventListener('click', async () => {
    try {
        const response = await fetch('/api/mode/validation', { method: 'POST' });
        if (response.ok) {
            showTestMode('Validation Mode', 'Place AprilTag EXACTLY 1.5m from camera, facing straight on. System will measure accuracy. Target: <2cm distance error, <2° angle error.');
        } else {
            alert('Failed to start validation mode');
        }
    } catch (err) {
        console.error('Error starting validation:', err);
        alert('Error starting validation mode');
    }
});

document.getElementById('start-diagnostics')?.addEventListener('click', async () => {
    try {
        const response = await fetch('/api/mode/diagnostics', { method: 'POST' });
        if (response.ok) {
            showTestMode('Diagnostics Mode', 'Running comprehensive diagnostics. Check console for detailed output.');
        } else {
            alert('Failed to start diagnostics');
        }
    } catch (err) {
        console.error('Error starting diagnostics:', err);
        alert('Error starting diagnostics');
    }
});

document.getElementById('stop-test-mode')?.addEventListener('click', async () => {
    try {
        const response = await fetch('/api/mode/normal', { method: 'POST' });
        if (response.ok) {
            hideTestMode();
        } else {
            alert('Failed to stop test mode');
        }
    } catch (err) {
        console.error('Error stopping test mode:', err);
        alert('Error stopping test mode');
    }
});

function showTestMode(modeName, instructions) {
    const statusEl = document.getElementById('test-mode-status');
    const modeEl = document.getElementById('current-test-mode');
    const instrEl = document.getElementById('test-instructions');
    const stopBtn = document.getElementById('stop-test-mode');

    modeEl.textContent = modeName;
    instrEl.textContent = instructions;
    statusEl.style.display = 'block';
    stopBtn.style.display = 'block';

    // Hide start buttons
    document.getElementById('start-calibration-mode').style.display = 'none';
    document.getElementById('start-validation-mode').style.display = 'none';
    document.getElementById('start-diagnostics').style.display = 'none';
}

function hideTestMode() {
    const statusEl = document.getElementById('test-mode-status');
    const stopBtn = document.getElementById('stop-test-mode');

    statusEl.style.display = 'none';
    stopBtn.style.display = 'none';

    // Show start buttons
    document.getElementById('start-calibration-mode').style.display = 'block';
    document.getElementById('start-validation-mode').style.display = 'block';
    document.getElementById('start-diagnostics').style.display = 'block';
}

// Auto-align controls
document.getElementById('set-align-target')?.addEventListener('click', async () => {
    const tagId = parseInt(document.getElementById('target-tag-id').value);

    if (isNaN(tagId) || tagId < 1 || tagId > 16) {
        alert('Please enter a valid tag ID (1-16)');
        return;
    }

    try {
        const response = await fetch('/api/autoalign/set_target', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ tag_id: tagId })
        });

        if (response.ok) {
            document.getElementById('autoalign-status').style.display = 'block';
            console.log(`Set auto-align target to tag ${tagId}`);
        } else {
            alert('Failed to set alignment target');
        }
    } catch (err) {
        console.error('Error setting align target:', err);
        alert('Error setting alignment target');
    }
});

document.getElementById('calculate-positions')?.addEventListener('click', async () => {
    const distance = parseFloat(document.getElementById('shooting-distance').value);
    const alliance = document.querySelector('input[name="alliance"]:checked').value;

    try {
        const response = await fetch('/api/autoalign/calculate_positions', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                shooting_distance: distance,
                is_red_alliance: alliance === 'red'
            })
        });

        if (response.ok) {
            const data = await response.json();
            displayShootingPositions(data.positions || []);
        } else {
            alert('Failed to calculate shooting positions');
        }
    } catch (err) {
        console.error('Error calculating positions:', err);
        alert('Error calculating shooting positions');
    }
});

function displayShootingPositions(positions) {
    const container = document.getElementById('positions-list');
    const wrapper = document.getElementById('shooting-positions');

    if (positions.length === 0) {
        container.innerHTML = '<p class="muted">No valid shooting positions found</p>';
        wrapper.style.display = 'block';
        return;
    }

    container.innerHTML = '';

    positions.slice(0, 5).forEach((pos, index) => {
        const card = document.createElement('div');
        card.className = `position-card ${pos.position_type}`;
        card.innerHTML = `
            <h5>Position ${index + 1}: ${pos.position_type.toUpperCase()}</h5>
            <div class="position-details">
                <span><strong>X:</strong> ${pos.pose.x.toFixed(2)}m</span>
                <span><strong>Y:</strong> ${pos.pose.y.toFixed(2)}m</span>
                <span><strong>θ:</strong> ${(pos.pose.theta * 180 / Math.PI).toFixed(1)}°</span>
                <span><strong>Distance:</strong> ${pos.distance_to_target_m.toFixed(2)}m</span>
                <span><strong>Visible Tags:</strong> ${pos.visible_tags}</span>
                <span><strong>Accuracy:</strong> ${(pos.expected_accuracy * 100).toFixed(0)}%</span>
            </div>
        `;
        card.addEventListener('click', () => {
            navigateToPosition(pos.pose);
        });
        container.appendChild(card);
    });

    wrapper.style.display = 'block';
}

function navigateToPosition(pose) {
    // Send navigation command to robot via NT
    fetch('/api/autoalign/navigate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(pose)
    }).then(response => {
        if (response.ok) {
            console.log('Navigation command sent:', pose);
        }
    }).catch(err => {
        console.error('Error sending navigation command:', err);
    });
}

// Update auto-align status from SSE
function updateAutoAlignStatus(data) {
    if (!data.autoalign) return;

    const status = data.autoalign;

    // Update status display
    const targetVisibleEl = document.getElementById('target-visible');
    if (targetVisibleEl) {
        targetVisibleEl.textContent = status.target_visible ? 'YES' : 'NO';
        targetVisibleEl.className = status.target_visible ? 'status-indicator connected' : 'status-indicator disconnected';
    }

    const distanceEl = document.getElementById('align-distance');
    if (distanceEl && status.distance_m !== undefined) {
        distanceEl.textContent = status.distance_m.toFixed(2);
    }

    const headingEl = document.getElementById('heading-error');
    if (headingEl && status.heading_error_deg !== undefined) {
        headingEl.textContent = status.heading_error_deg.toFixed(1);
    }

    const stageEl = document.getElementById('align-stage');
    if (stageEl && status.current_stage) {
        stageEl.textContent = status.current_stage.replace('_', ' ').toUpperCase();
        stageEl.className = `badge ${status.current_stage}`;
    }

    const readyEl = document.getElementById('ready-to-shoot');
    if (readyEl) {
        readyEl.textContent = status.ready_to_shoot ? 'YES' : 'NO';
        readyEl.className = status.ready_to_shoot ? 'status-indicator connected' : 'status-indicator disconnected';
    }
}

// Hook into existing SSE update
const originalUpdateStatus = app.updateStatus.bind(app);
app.updateStatus = function(data) {
    originalUpdateStatus(data);
    updateAutoAlignStatus(data);
};

console.log('Phase 3 auto-align & testing controls initialized');
