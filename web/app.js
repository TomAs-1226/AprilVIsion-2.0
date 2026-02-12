/**
 * AprilVision 3.2 - FRC Competition Vision System
 * Minimal helper module for external script usage.
 *
 * The main SPA logic resides in index.html (AprilVisionApp class).
 * This file provides lightweight utility functions that can be
 * imported by other pages or scripts if needed.
 *
 * Built by Team 1226
 */

const AVHelpers = {
    /**
     * Check if the AprilVision engine API is reachable.
     * Returns a promise resolving to { online: boolean, latency: number }.
     */
    async pingEngine() {
        const start = performance.now();
        try {
            const resp = await fetch('/api/av/status', {
                signal: AbortSignal.timeout(3000)
            });
            const latency = Math.round(performance.now() - start);
            return { online: resp.ok, latency: latency };
        } catch (e) {
            const latency = Math.round(performance.now() - start);
            return { online: false, latency: latency };
        }
    },

    /**
     * Fetch the list of detected cameras from the AV API.
     * Returns a promise resolving to an array of camera objects.
     */
    async getCameras() {
        try {
            const resp = await fetch('/api/av/cameras', {
                signal: AbortSignal.timeout(5000)
            });
            if (resp.ok) {
                const data = await resp.json();
                return data.cameras || [];
            }
        } catch (e) {
            // API unavailable
        }
        return [];
    },

    /**
     * Get the MJPEG stream URL for a given camera index and type.
     * @param {number} index - Camera index
     * @param {string} type - 'input' or 'output'
     * @returns {string} Stream URL
     */
    getStreamUrl(index, type) {
        return '/stream/' + index + '/' + (type === 'input' ? 'input' : 'output');
    },

    /**
     * Format seconds into a human-readable uptime string.
     * @param {number} seconds
     * @returns {string}
     */
    formatUptime(seconds) {
        if (typeof seconds !== 'number' || seconds < 0) return '--';
        const s = Math.floor(seconds);
        if (s < 60) return s + 's';
        const m = Math.floor(s / 60);
        const rs = s % 60;
        if (m < 60) return m + 'm ' + rs + 's';
        const h = Math.floor(m / 60);
        const rm = m % 60;
        return h + 'h ' + rm + 'm';
    }
};
