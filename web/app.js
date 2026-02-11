/**
 * AprilVision 2.0 - Custom Vision System
 * Built on PhotonVision libraries
 *
 * This file is included as a landing page helper.
 * The main dashboard functionality comes from PhotonVision's
 * built-in web interface, accessed through the reverse proxy
 * which rebrands it as AprilVision 2.0.
 */

// Landing page status checker
class AprilVisionStatus {
    constructor() {
        this.pvConnected = false;
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
        } catch {
            this.pvConnected = false;
        }

        this.updateStatusDisplay();
    }

    updateStatusDisplay() {
        const statusEl = document.getElementById('pv-status');
        if (statusEl) {
            if (this.pvConnected) {
                statusEl.textContent = 'Online';
                statusEl.classList.remove('error');
            } else {
                statusEl.textContent = 'Starting...';
                statusEl.classList.add('error');
            }
        }
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.aprilVisionStatus = new AprilVisionStatus();
});
