#!/bin/bash
#===============================================================================
# AprilVision 2.0 - Pre-Match System Health Check
#
# Run this before every match to verify all vision components are operational.
# Exit code 0 = all systems go, non-zero = issues found.
#
# Usage: ./scripts/health_check.sh [--quiet]
#===============================================================================

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

QUIET=false
ISSUES=0

if [[ "$1" == "--quiet" ]]; then
    QUIET=true
fi

pass() {
    if ! $QUIET; then
        printf "  %-28s ${GREEN}%s${NC}\n" "$1" "$2"
    fi
}

warn() {
    if ! $QUIET; then
        printf "  %-28s ${YELLOW}%s${NC}\n" "$1" "$2"
    fi
    ISSUES=$((ISSUES + 1))
}

fail() {
    if ! $QUIET; then
        printf "  %-28s ${RED}%s${NC}\n" "$1" "$2"
    fi
    ISSUES=$((ISSUES + 1))
}

if ! $QUIET; then
    echo ""
    echo -e "${CYAN}AprilVision 2.0 - System Health Check${NC}"
    echo "======================================"
fi

# --- PhotonVision Service ---
if systemctl is-active --quiet photonvision 2>/dev/null; then
    pass "PhotonVision Service" "RUNNING"
else
    fail "PhotonVision Service" "NOT RUNNING"
fi

# --- Dashboard Service ---
if systemctl is-active --quiet aprilvision-dashboard 2>/dev/null; then
    pass "Dashboard Proxy" "RUNNING"
else
    fail "Dashboard Proxy" "NOT RUNNING"
fi

# --- Java ---
if command -v java &>/dev/null; then
    JAVA_VER=$(java -version 2>&1 | head -n 1 | awk -F '"' '{print $2}')
    JAVA_MAJOR=$(echo "$JAVA_VER" | cut -d'.' -f1)
    if [[ "$JAVA_MAJOR" == "17" ]]; then
        pass "Java 17" "OK ($JAVA_VER)"
    else
        warn "Java" "WRONG VERSION ($JAVA_VER, need 17)"
    fi
else
    fail "Java" "NOT INSTALLED"
fi

# --- PhotonVision JAR ---
PV_JAR="/opt/photonvision/photonvision.jar"
if [[ -f "$PV_JAR" ]]; then
    JAR_SIZE=$(du -h "$PV_JAR" | awk '{print $1}')
    pass "PhotonVision JAR" "OK ($JAR_SIZE)"
else
    fail "PhotonVision JAR" "MISSING"
fi

# --- Camera Devices ---
if command -v v4l2-ctl &>/dev/null; then
    CAM_COUNT=$(ls /dev/video* 2>/dev/null | wc -l)
    if [[ $CAM_COUNT -gt 0 ]]; then
        pass "Camera Devices" "$CAM_COUNT found"
    else
        warn "Camera Devices" "NONE FOUND"
    fi
else
    warn "Camera Devices" "v4l2-ctl not installed"
fi

# --- Network (roboRIO) ---
CONFIG_FILE="$(dirname "$(dirname "$0")")/config/config.yml"
if [[ -f "$CONFIG_FILE" ]]; then
    NT_SERVER=$(grep "nt_server" "$CONFIG_FILE" | sed 's/.*"\(.*\)".*/\1/')
    if [[ -n "$NT_SERVER" && "$NT_SERVER" != "10.00.00.2" ]]; then
        if ping -c 1 -W 2 "$NT_SERVER" &>/dev/null; then
            pass "Network (roboRIO)" "REACHABLE ($NT_SERVER)"
        else
            warn "Network (roboRIO)" "UNREACHABLE ($NT_SERVER)"
        fi
    else
        warn "Network (roboRIO)" "NOT CONFIGURED"
    fi
else
    warn "Network (roboRIO)" "CONFIG NOT FOUND"
fi

# --- PhotonVision HTTP ---
if command -v curl &>/dev/null; then
    if curl -s --max-time 3 http://localhost:5800 &>/dev/null; then
        pass "PhotonVision HTTP" "RESPONDING (:5800)"
    else
        warn "PhotonVision HTTP" "NOT RESPONDING"
    fi
fi

# --- Dashboard HTTP ---
if command -v curl &>/dev/null; then
    if curl -s --max-time 3 http://localhost:5801 &>/dev/null; then
        pass "Dashboard HTTP" "RESPONDING (:5801)"
    else
        warn "Dashboard HTTP" "NOT RESPONDING"
    fi
fi

# --- Disk Space ---
DISK_FREE=$(df -h / | awk 'NR==2 {print $4}')
DISK_PCT=$(df / | awk 'NR==2 {print $5}' | tr -d '%')
if [[ $DISK_PCT -lt 90 ]]; then
    pass "Disk Space" "OK ($DISK_FREE free)"
else
    warn "Disk Space" "LOW ($DISK_FREE free, ${DISK_PCT}% used)"
fi

# --- Memory ---
MEM_USED=$(free -m | awk 'NR==2 {print $3}')
MEM_TOTAL=$(free -m | awk 'NR==2 {print $2}')
MEM_PCT=$((MEM_USED * 100 / MEM_TOTAL))
if [[ $MEM_PCT -lt 85 ]]; then
    pass "Memory" "OK (${MEM_USED}M / ${MEM_TOTAL}M)"
else
    warn "Memory" "HIGH (${MEM_USED}M / ${MEM_TOTAL}M, ${MEM_PCT}%)"
fi

# --- CPU Temperature (if available) ---
if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
    TEMP_RAW=$(cat /sys/class/thermal/thermal_zone0/temp)
    TEMP_C=$((TEMP_RAW / 1000))
    if [[ $TEMP_C -lt 75 ]]; then
        pass "CPU Temperature" "OK (${TEMP_C}C)"
    elif [[ $TEMP_C -lt 85 ]]; then
        warn "CPU Temperature" "WARM (${TEMP_C}C)"
    else
        fail "CPU Temperature" "HOT (${TEMP_C}C) - THROTTLING LIKELY"
    fi
fi

# --- Results ---
if ! $QUIET; then
    echo "--------------------------------------"
    if [[ $ISSUES -eq 0 ]]; then
        echo -e "  ${GREEN}RESULT: ALL SYSTEMS GO${NC}"
    else
        echo -e "  ${YELLOW}RESULT: $ISSUES issue(s) found${NC}"
    fi
    echo ""
fi

exit $ISSUES
