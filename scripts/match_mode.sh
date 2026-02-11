#!/bin/bash
#===============================================================================
# AprilVision 2.0 - Match Mode
#
# Optimizes the coprocessor for competition performance.
# Enable before matches, disable for practice/development.
#
# Usage:
#   ./scripts/match_mode.sh enable    # Competition settings
#   ./scripts/match_mode.sh disable   # Normal settings
#   ./scripts/match_mode.sh status    # Show current mode
#===============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

STATE_FILE="/tmp/aprilvision-match-mode"
LOG_DIR="/opt/aprilvision/logs"

log_info() { echo -e "${BLUE}[MATCH]${NC} $1"; }
log_ok()   { echo -e "${GREEN}[MATCH]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[MATCH]${NC} $1"; }

enable_match_mode() {
    echo ""
    echo -e "${CYAN}AprilVision 2.0 - Enabling Match Mode${NC}"
    echo "======================================"

    # Set CPU governor to performance
    if [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
        for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo performance | sudo tee "$cpu" > /dev/null 2>/dev/null || true
        done
        log_ok "CPU governor set to performance"
    else
        log_warn "CPU governor not available (may already be performance)"
    fi

    # Increase PhotonVision heap
    PV_SERVICE="/etc/systemd/system/photonvision.service"
    if [[ -f "$PV_SERVICE" ]]; then
        if grep -q "Xmx512m" "$PV_SERVICE"; then
            sudo sed -i 's/-Xmx512m/-Xmx768m/' "$PV_SERVICE"
            sudo systemctl daemon-reload
            log_ok "Java heap increased to 768MB"
            log_info "Restarting PhotonVision with new memory settings..."
            sudo systemctl restart photonvision
            sleep 3
            sudo systemctl restart aprilvision-dashboard
            log_ok "Services restarted"
        else
            log_info "Heap already configured for match mode"
        fi
    fi

    # Set process priority
    PV_PID=$(pgrep -f "photonvision.jar" 2>/dev/null || true)
    if [[ -n "$PV_PID" ]]; then
        sudo renice -n -10 -p "$PV_PID" > /dev/null 2>&1 || true
        log_ok "PhotonVision process priority raised (PID $PV_PID)"
    fi

    # Reduce kernel swappiness
    sudo sysctl -w vm.swappiness=10 > /dev/null 2>&1 || true
    log_ok "Kernel swappiness reduced to 10"

    # Disable unnecessary services (safe list only)
    for svc in cups bluetooth ModemManager; do
        if systemctl is-active --quiet "$svc" 2>/dev/null; then
            sudo systemctl stop "$svc" 2>/dev/null || true
            log_ok "Stopped $svc to free resources"
        fi
    done

    # Record state
    echo "enabled" > "$STATE_FILE"
    sudo mkdir -p "$LOG_DIR"
    echo "$(date '+%Y-%m-%d %H:%M:%S') match_mode=enabled" | sudo tee -a "$LOG_DIR/match_mode.log" > /dev/null

    echo "--------------------------------------"
    echo -e "  ${GREEN}MATCH MODE: ENABLED${NC}"
    echo "  CPU: performance | Heap: 768MB"
    echo "  Swap: low | Priority: high"
    echo ""
    echo -e "  ${YELLOW}Remember: ./scripts/match_mode.sh disable${NC}"
    echo -e "  ${YELLOW}after practice to restore normal settings${NC}"
    echo ""
}

disable_match_mode() {
    echo ""
    echo -e "${CYAN}AprilVision 2.0 - Disabling Match Mode${NC}"
    echo "======================================="

    # Reset CPU governor
    if [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
        for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo ondemand | sudo tee "$cpu" > /dev/null 2>/dev/null || true
        done
        log_ok "CPU governor reset to ondemand"
    fi

    # Restore PhotonVision heap
    PV_SERVICE="/etc/systemd/system/photonvision.service"
    if [[ -f "$PV_SERVICE" ]]; then
        if grep -q "Xmx768m" "$PV_SERVICE"; then
            sudo sed -i 's/-Xmx768m/-Xmx512m/' "$PV_SERVICE"
            sudo systemctl daemon-reload
            log_ok "Java heap restored to 512MB"
            log_info "Restarting PhotonVision with normal memory settings..."
            sudo systemctl restart photonvision
            sleep 3
            sudo systemctl restart aprilvision-dashboard
            log_ok "Services restarted"
        fi
    fi

    # Restore swappiness
    sudo sysctl -w vm.swappiness=60 > /dev/null 2>&1 || true
    log_ok "Kernel swappiness restored to 60"

    # Record state
    echo "disabled" > "$STATE_FILE"
    sudo mkdir -p "$LOG_DIR"
    echo "$(date '+%Y-%m-%d %H:%M:%S') match_mode=disabled" | sudo tee -a "$LOG_DIR/match_mode.log" > /dev/null

    echo "--------------------------------------"
    echo -e "  ${BLUE}MATCH MODE: DISABLED${NC}"
    echo "  Normal settings restored."
    echo ""
}

show_status() {
    echo ""
    echo -e "${CYAN}AprilVision 2.0 - Match Mode Status${NC}"
    echo "====================================="

    if [[ -f "$STATE_FILE" ]] && grep -q "enabled" "$STATE_FILE"; then
        echo -e "  Current mode: ${GREEN}MATCH (competition)${NC}"
    else
        echo -e "  Current mode: ${BLUE}NORMAL (practice)${NC}"
    fi

    # Show CPU governor
    if [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
        GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)
        echo "  CPU governor: $GOV"
    fi

    # Show PV heap
    PV_SERVICE="/etc/systemd/system/photonvision.service"
    if [[ -f "$PV_SERVICE" ]]; then
        HEAP=$(grep -o 'Xmx[0-9]*m' "$PV_SERVICE" 2>/dev/null || echo "unknown")
        echo "  Java heap:    $HEAP"
    fi

    # Show swappiness
    SWAP=$(cat /proc/sys/vm/swappiness 2>/dev/null || echo "unknown")
    echo "  Swappiness:   $SWAP"

    # Show recent log
    if [[ -f "$LOG_DIR/match_mode.log" ]]; then
        echo ""
        echo "  Recent changes:"
        tail -5 "$LOG_DIR/match_mode.log" | while read -r line; do
            echo "    $line"
        done
    fi
    echo ""
}

case "${1:-}" in
    enable)  enable_match_mode ;;
    disable) disable_match_mode ;;
    status)  show_status ;;
    *)
        echo "Usage: $0 {enable|disable|status}"
        echo ""
        echo "  enable   - Optimize for competition (low latency, high performance)"
        echo "  disable  - Restore normal settings (power saving, development)"
        echo "  status   - Show current match mode state"
        exit 1
        ;;
esac
