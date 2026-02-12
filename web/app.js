/**
 * AprilVision 3.2 - FRC Competition Vision System
 * Dashboard status monitor, camera detection, and health checker
 *
 * Built by Team 1226
 */

class AprilVisionStatus {
    constructor() {
        this.engineConnected = false;
        this.lastCheckTime = null;
        this.consecutiveFailures = 0;
        this.checkInterval = null;
        this.cameraCount = 0;
        this.init();
    }

    init() {
        this.checkEngineStatus();
        this.checkInterval = setInterval(() => {
            this.checkEngineStatus();
        }, 5000);
    }

    async checkEngineStatus() {
        try {
            const response = await fetch('/api/settings/general', {
                signal: AbortSignal.timeout(3000)
            });
            this.engineConnected = response.ok;
            if (response.ok) {
                this.consecutiveFailures = 0;
            }
        } catch {
            this.engineConnected = false;
            this.consecutiveFailures++;
        }

        // Also check cameras via custom API
        try {
            const camResp = await fetch('/api/aprilvision/cameras', {
                signal: AbortSignal.timeout(3000)
            });
            if (camResp.ok) {
                const data = await camResp.json();
                this.cameraCount = data.count || 0;
            }
        } catch {
            // Camera API may not be available yet
        }

        this.lastCheckTime = new Date();
        this.updateStatusDisplay();
    }

    updateStatusDisplay() {
        const statusEl = document.getElementById('pv-status');
        const dotEl = document.getElementById('pv-dot');

        if (statusEl) {
            if (this.engineConnected) {
                statusEl.textContent = 'Online';
                if (dotEl) dotEl.className = 'status-dot green';
            } else if (this.consecutiveFailures > 3) {
                statusEl.textContent = 'Offline';
                if (dotEl) dotEl.className = 'status-dot red';
            } else {
                statusEl.textContent = 'Starting...';
                if (dotEl) dotEl.className = 'status-dot yellow';
            }
        }
    }

    isOnline() {
        return this.engineConnected;
    }

    getCameraCount() {
        return this.cameraCount;
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.aprilVisionStatus = new AprilVisionStatus();
});
