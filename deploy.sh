#!/bin/bash
# =============================================================================
# AprilVision 3.2 - Quick Deploy
# Shortcut for: sudo ./setup.sh --deploy [--team TEAM]
#
# This copies bridge + dashboard files to /opt/aprilvision and restarts
# the service. For full setup (engine, Java, etc.) use setup.sh directly.
#
# Usage:
#   sudo ./deploy.sh               # Redeploy files + restart
#   sudo ./deploy.sh --team 5805   # Redeploy + update team number
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/setup.sh" --deploy "$@"
