/**
 * AprilVision 2.0 - FRC Competition Vision System
 * Dashboard status monitor and health checker
 *
 * Built by Team 1226
 * Detection engine: PhotonVision
 */

class AprilVisionStatus {
    constructor() {
        this.pvConnected = false;
        this.lastCheckTime = null;
        this.consecutiveFailures = 0;
        this.checkInterval = null;
        this.init();
    }

    init() {
        this.checkPhotonVisionStatus();
        this.checkInterval = setInterval(() => {
            this.checkPhotonVisionStatus();
        }, 5000);
    }

    async checkPhotonVisionStatus() {
        try {
            const response = await fetch('/api/settings/general', {
                signal: AbortSignal.timeout(3000)
            });
            this.pvConnected = response.ok;
            if (response.ok) {
                this.consecutiveFailures = 0;
            }
        } catch {
            this.pvConnected = false;
            this.consecutiveFailures++;
        }

        this.lastCheckTime = new Date();
        this.updateStatusDisplay();
    }

    updateStatusDisplay() {
        const statusEl = document.getElementById('pv-status');
        const dotEl = document.getElementById('pv-dot');

        if (statusEl) {
            if (this.pvConnected) {
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
        return this.pvConnected;
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.aprilVisionStatus = new AprilVisionStatus();
});
