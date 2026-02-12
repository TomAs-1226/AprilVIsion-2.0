#!/bin/bash
#===============================================================================
# AprilVision 3.2 - Network Diagnostics
#
# Tests network connectivity, port availability, and camera bandwidth
# for the AprilVision vision system.
#
# Usage: ./scripts/network_diagnostics.sh
#===============================================================================

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

ISSUES=0

pass() {
    printf "  %-32s ${GREEN}%s${NC}\n" "$1" "$2"
}

warn() {
    printf "  %-32s ${YELLOW}%s${NC}\n" "$1" "$2"
    ISSUES=$((ISSUES + 1))
}

fail() {
    printf "  %-32s ${RED}%s${NC}\n" "$1" "$2"
    ISSUES=$((ISSUES + 1))
}

info() {
    printf "  %-32s ${BLUE}%s${NC}\n" "$1" "$2"
}

echo ""
echo -e "${CYAN}AprilVision 3.2 - Network Diagnostics${NC}"
echo "========================================"
echo ""

# =========================================================================
# Section 1: Network Interfaces
# =========================================================================
echo -e "${BLUE}--- Network Interfaces ---${NC}"

# Get primary IP
IP_ADDR=$(hostname -I 2>/dev/null | awk '{print $1}')
if [[ -n "$IP_ADDR" ]]; then
    pass "Primary IP" "$IP_ADDR"
else
    fail "Primary IP" "NO IP ADDRESS FOUND"
fi

# Check if we have a link
if ip link show 2>/dev/null | grep -q "state UP"; then
    IFACE=$(ip link show 2>/dev/null | grep "state UP" | head -1 | awk -F: '{print $2}' | tr -d ' ')
    pass "Network Interface" "$IFACE (UP)"
else
    fail "Network Interface" "NO ACTIVE INTERFACE"
fi

# Check hostname
HOSTNAME=$(hostname 2>/dev/null || echo "unknown")
info "Hostname" "$HOSTNAME"

echo ""

# =========================================================================
# Section 2: roboRIO Connectivity
# =========================================================================
echo -e "${BLUE}--- roboRIO Connectivity ---${NC}"

CONFIG_FILE="$(dirname "$(dirname "$0")")/config/config.yml"
NT_SERVER=""
TEAM_NUMBER=""

if [[ -f "$CONFIG_FILE" ]]; then
    NT_SERVER=$(grep "nt_server" "$CONFIG_FILE" 2>/dev/null | sed 's/.*"\(.*\)".*/\1/')
    TEAM_NUMBER=$(grep "team_number" "$CONFIG_FILE" 2>/dev/null | awk '{print $2}')

    if [[ -n "$TEAM_NUMBER" && "$TEAM_NUMBER" != "0000" ]]; then
        info "Team Number" "$TEAM_NUMBER"
    else
        warn "Team Number" "NOT CONFIGURED"
    fi

    if [[ -n "$NT_SERVER" && "$NT_SERVER" != "10.00.00.2" ]]; then
        info "roboRIO IP" "$NT_SERVER"

        # Ping test
        if ping -c 3 -W 2 "$NT_SERVER" &>/dev/null; then
            # Get latency
            LATENCY=$(ping -c 3 -W 2 "$NT_SERVER" 2>/dev/null | tail -1 | awk -F'/' '{print $5}')
            if [[ -n "$LATENCY" ]]; then
                # Check if latency is acceptable
                LATENCY_INT=$(echo "$LATENCY" | cut -d'.' -f1)
                if [[ "$LATENCY_INT" -lt 5 ]]; then
                    pass "Ping roboRIO" "${LATENCY}ms (excellent)"
                elif [[ "$LATENCY_INT" -lt 20 ]]; then
                    pass "Ping roboRIO" "${LATENCY}ms (good)"
                elif [[ "$LATENCY_INT" -lt 50 ]]; then
                    warn "Ping roboRIO" "${LATENCY}ms (high - check connection)"
                else
                    fail "Ping roboRIO" "${LATENCY}ms (very high!)"
                fi
            else
                pass "Ping roboRIO" "REACHABLE"
            fi
        else
            fail "Ping roboRIO" "UNREACHABLE ($NT_SERVER)"
        fi

        # Try mDNS resolution
        MDNS_HOST="roboRIO-${TEAM_NUMBER}-FRC.local"
        if command -v getent &>/dev/null; then
            if getent hosts "$MDNS_HOST" &>/dev/null; then
                MDNS_IP=$(getent hosts "$MDNS_HOST" 2>/dev/null | awk '{print $1}')
                pass "mDNS ($MDNS_HOST)" "$MDNS_IP"
            else
                warn "mDNS ($MDNS_HOST)" "NOT RESOLVED"
            fi
        fi

        # NetworkTables port test (1735 is the default NT4 port)
        if command -v nc &>/dev/null || command -v ncat &>/dev/null; then
            NC_CMD="nc"
            command -v ncat &>/dev/null && NC_CMD="ncat"
            if $NC_CMD -z -w 2 "$NT_SERVER" 1735 2>/dev/null; then
                pass "NetworkTables (port 1735)" "OPEN"
            else
                warn "NetworkTables (port 1735)" "CLOSED or FILTERED"
            fi
        else
            info "NetworkTables (port 1735)" "nc not installed, skipping"
        fi
    else
        warn "roboRIO IP" "NOT CONFIGURED (run ./setup.sh --team XXXX)"
    fi
else
    warn "Config File" "NOT FOUND ($CONFIG_FILE)"
fi

echo ""

# =========================================================================
# Section 3: Vision System Ports
# =========================================================================
echo -e "${BLUE}--- Vision System Ports ---${NC}"

# Port 5800 - Detection Engine
if command -v curl &>/dev/null; then
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --max-time 3 http://localhost:5800 2>/dev/null)
    if [[ "$HTTP_CODE" -gt 0 && "$HTTP_CODE" -lt 500 ]]; then
        pass "Port 5800 (Engine)" "RESPONDING (HTTP $HTTP_CODE)"
    else
        fail "Port 5800 (Engine)" "NOT RESPONDING"
    fi
else
    # Fallback to checking if port is listening
    if ss -tlnp 2>/dev/null | grep -q ":5800 "; then
        pass "Port 5800 (Engine)" "LISTENING"
    else
        fail "Port 5800 (Engine)" "NOT LISTENING"
    fi
fi

# Port 5801 - Dashboard
if command -v curl &>/dev/null; then
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --max-time 3 http://localhost:5801 2>/dev/null)
    if [[ "$HTTP_CODE" -gt 0 && "$HTTP_CODE" -lt 500 ]]; then
        pass "Port 5801 (Dashboard)" "RESPONDING (HTTP $HTTP_CODE)"
    else
        fail "Port 5801 (Dashboard)" "NOT RESPONDING"
    fi
else
    if ss -tlnp 2>/dev/null | grep -q ":5801 "; then
        pass "Port 5801 (Dashboard)" "LISTENING"
    else
        fail "Port 5801 (Dashboard)" "NOT LISTENING"
    fi
fi

# Check if ports are accessible externally
if [[ -n "$IP_ADDR" ]]; then
    if command -v curl &>/dev/null; then
        EXT_CODE=$(curl -s -o /dev/null -w "%{http_code}" --max-time 3 "http://${IP_ADDR}:5801" 2>/dev/null)
        if [[ "$EXT_CODE" -gt 0 && "$EXT_CODE" -lt 500 ]]; then
            pass "External access (:5801)" "OK from $IP_ADDR"
        else
            warn "External access (:5801)" "MAY BE BLOCKED (firewall?)"
        fi
    fi
fi

echo ""

# =========================================================================
# Section 4: Camera USB Bandwidth
# =========================================================================
echo -e "${BLUE}--- Camera USB Bandwidth ---${NC}"

if command -v lsusb &>/dev/null; then
    # Count USB video devices
    USB_CAMS=$(lsusb 2>/dev/null | grep -i -c "camera\|webcam\|video\|uvc" || echo "0")
    info "USB Camera Devices" "$USB_CAMS detected"

    # Check USB controller versions
    USB3_COUNT=$(lsusb -t 2>/dev/null | grep -c "5000M\|10000M\|20000M" || echo "0")
    USB2_COUNT=$(lsusb -t 2>/dev/null | grep -c "480M" || echo "0")

    if [[ "$USB3_COUNT" -gt 0 ]]; then
        pass "USB 3.0 Controllers" "$USB3_COUNT available"
    else
        warn "USB 3.0 Controllers" "NONE (cameras limited to USB 2.0 bandwidth)"
    fi

    # Check for bandwidth warnings
    CAM_COUNT=$(ls /dev/video* 2>/dev/null | wc -l)
    if [[ $CAM_COUNT -gt 4 ]]; then
        warn "Video Devices" "$CAM_COUNT found - check for duplicate endpoints"
    elif [[ $CAM_COUNT -gt 0 ]]; then
        pass "Video Devices" "$CAM_COUNT found"
    else
        warn "Video Devices" "NONE FOUND"
    fi

    # USB bandwidth estimate
    if [[ $CAM_COUNT -gt 0 ]]; then
        # At 640x480 @ 30fps MJPEG, each camera uses ~15-30 MB/s
        # USB 2.0 total bandwidth is ~35 MB/s usable
        # USB 3.0 total bandwidth is ~400 MB/s usable
        ACTIVE_CAMS=$(ls /dev/video* 2>/dev/null | wc -l)
        # Each video device pair = 1 camera typically
        EST_CAMS=$(( (ACTIVE_CAMS + 1) / 2 ))

        if [[ "$USB3_COUNT" -eq 0 && "$EST_CAMS" -gt 2 ]]; then
            warn "Bandwidth Estimate" "$EST_CAMS cameras on USB 2.0 may cause issues"
            echo -e "    ${YELLOW}Tip: Use USB 3.0 hub or reduce resolution/FPS${NC}"
        else
            pass "Bandwidth Estimate" "OK for $EST_CAMS camera(s)"
        fi
    fi
else
    info "USB Information" "lsusb not installed, skipping"
fi

echo ""

# =========================================================================
# Section 5: Firewall Check
# =========================================================================
echo -e "${BLUE}--- Firewall Status ---${NC}"

if command -v ufw &>/dev/null; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | head -1)
    if echo "$UFW_STATUS" | grep -qi "inactive"; then
        pass "UFW Firewall" "INACTIVE (no restrictions)"
    elif echo "$UFW_STATUS" | grep -qi "active"; then
        warn "UFW Firewall" "ACTIVE - verify ports 5800,5801 are allowed"
        echo -e "    ${YELLOW}Run: sudo ufw allow 5800/tcp && sudo ufw allow 5801/tcp${NC}"
    fi
elif command -v iptables &>/dev/null; then
    RULE_COUNT=$(sudo iptables -L INPUT 2>/dev/null | wc -l)
    if [[ "$RULE_COUNT" -le 3 ]]; then
        pass "iptables" "NO RESTRICTIONS"
    else
        info "iptables" "$((RULE_COUNT - 2)) rules found - verify port access"
    fi
else
    info "Firewall" "No firewall tool detected"
fi

echo ""

# =========================================================================
# Results Summary
# =========================================================================
echo "========================================"
if [[ $ISSUES -eq 0 ]]; then
    echo -e "  ${GREEN}NETWORK: ALL CHECKS PASSED${NC}"
else
    echo -e "  ${YELLOW}NETWORK: $ISSUES issue(s) found${NC}"
fi
echo ""

if [[ $ISSUES -gt 0 ]]; then
    echo -e "${BLUE}Common Fixes:${NC}"
    echo "  - Verify Ethernet cable is connected"
    echo "  - Ensure robot radio is powered on"
    echo "  - Set team number: ./setup.sh --team YOUR_TEAM_NUMBER"
    echo "  - Check firewall: sudo ufw allow 5800/tcp && sudo ufw allow 5801/tcp"
    echo "  - For USB bandwidth: use USB 3.0 ports or reduce camera count"
    echo ""
fi

exit $ISSUES
