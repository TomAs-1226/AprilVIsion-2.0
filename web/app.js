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
            this.drawField(pose);
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

        // NT status (inferred from fused pose validity)
        const ntStatus = document.getElementById('status-nt');
        if (data.fused_valid) {
            ntStatus.textContent = 'NT: Connected';
            ntStatus.classList.remove('disconnected');
            ntStatus.classList.add('connected');
        } else {
            ntStatus.textContent = 'NT: Disconnected';
            ntStatus.classList.remove('connected');
            ntStatus.classList.add('disconnected');
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

            // Draw margin
            ctx.fillStyle = '#ffffff';
            ctx.font = '10px sans-serif';
            ctx.fillText(`M:${tag.margin.toFixed(0)}`, center[0] * scaleX, center[1] * scaleY + 20);
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

    drawField(pose = null) {
        if (!this.fieldCtx) return;

        const ctx = this.fieldCtx;
        const width = this.fieldCanvas.width;
        const height = this.fieldCanvas.height;

        // Field dimensions (meters)
        const FIELD_LENGTH = 16.54;
        const FIELD_WIDTH = 8.21;

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
                `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)})`,
                offsetX + 5, offsetY + 12
            );
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
