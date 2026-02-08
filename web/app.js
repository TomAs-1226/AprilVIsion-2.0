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
        this.NUM_CAMERAS = 3;

        this.init();
    }

    init() {
        // Start camera streams immediately with auto-reconnect
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

        console.log('Vision Dashboard initialized');
    }

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

        // Force-set the MJPEG stream src with a cache-buster to ensure fresh connection
        const streamUrl = `/cam${camId}.mjpeg?t=${Date.now()}`;
        img.src = streamUrl;

        // On successful load, mark the panel as active
        img.onload = () => {
            const panel = img.closest('.camera-panel');
            if (panel) {
                panel.classList.remove('stream-error');
                panel.classList.add('stream-active');
            }
        };

        // On error, retry after a delay
        img.onerror = () => {
            console.warn(`Camera ${camId} stream failed, retrying...`);
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
    }

    connectEventSource() {
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
            console.log('SSE connection error, reconnecting...');
            setTimeout(() => this.connectEventSource(), 2000);
        };
    }

    setupCanvasOverlays() {
        for (let i = 0; i < 3; i++) {
            const img = document.getElementById(`cam${i}-img`);
            const canvas = document.getElementById(`cam${i}-overlay`);

            if (img && canvas) {
                // Resize canvas to match image
                const resizeCanvas = () => {
                    canvas.width = img.offsetWidth;
                    canvas.height = img.offsetHeight;
                };

                img.onload = resizeCanvas;
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
        for (let i = 0; i < 3; i++) {
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
