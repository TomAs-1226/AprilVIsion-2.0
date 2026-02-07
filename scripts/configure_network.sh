#!/bin/bash
#===============================================================================
# FRC Vision Coprocessor - Network Configuration
# Sets up static IP for FRC robot network
#
# Usage: sudo ./configure_network.sh <team_number>
# Example: sudo ./configure_network.sh 5805
#===============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [[ $EUID -ne 0 ]]; then
    echo -e "${RED}This script must be run as root (sudo)${NC}"
    exit 1
fi

if [[ -z "$1" ]]; then
    echo "Usage: sudo $0 <team_number>"
    echo "Example: sudo $0 5805"
    exit 1
fi

TEAM_NUMBER=$1

# Calculate IP addresses from team number
if [[ ${#TEAM_NUMBER} -le 2 ]]; then
    SUBNET="10.0.${TEAM_NUMBER}"
elif [[ ${#TEAM_NUMBER} -eq 3 ]]; then
    SUBNET="10.${TEAM_NUMBER:0:1}.${TEAM_NUMBER:1:2}"
else
    SUBNET="10.${TEAM_NUMBER:0:2}.${TEAM_NUMBER:2:2}"
fi

STATIC_IP="${SUBNET}.11"
GATEWAY="${SUBNET}.1"
ROBORIO_IP="${SUBNET}.2"

echo "==========================================="
echo "FRC Network Configuration"
echo "==========================================="
echo "Team Number:    $TEAM_NUMBER"
echo "Static IP:      $STATIC_IP"
echo "Gateway:        $GATEWAY"
echo "roboRIO IP:     $ROBORIO_IP"
echo "==========================================="

# Detect network interface (usually eth0 or end0 on Orange Pi)
INTERFACE=$(ip route | grep default | awk '{print $5}' | head -1)
if [[ -z "$INTERFACE" ]]; then
    # Try to find any ethernet interface
    INTERFACE=$(ip link | grep -E "^[0-9]+: (eth|end|enp)" | head -1 | cut -d: -f2 | tr -d ' ')
fi

if [[ -z "$INTERFACE" ]]; then
    echo -e "${RED}Could not detect network interface${NC}"
    echo "Available interfaces:"
    ip link show
    exit 1
fi

echo "Network interface: $INTERFACE"

# Check if using NetworkManager or netplan
if command -v nmcli &> /dev/null && systemctl is-active --quiet NetworkManager; then
    echo -e "${GREEN}Using NetworkManager${NC}"

    # Configure static IP via NetworkManager
    nmcli con mod "Wired connection 1" ipv4.addresses "${STATIC_IP}/24" 2>/dev/null || \
    nmcli con mod "${INTERFACE}" ipv4.addresses "${STATIC_IP}/24" 2>/dev/null || \
    nmcli con add type ethernet con-name "FRC-Static" ifname "$INTERFACE" ipv4.addresses "${STATIC_IP}/24" ipv4.gateway "$GATEWAY" ipv4.method manual

    nmcli con mod "FRC-Static" ipv4.gateway "$GATEWAY" 2>/dev/null || true
    nmcli con mod "FRC-Static" ipv4.method manual 2>/dev/null || true
    nmcli con mod "FRC-Static" ipv4.dns "" 2>/dev/null || true  # No DNS needed on robot

    # Restart connection
    nmcli con down "FRC-Static" 2>/dev/null || true
    nmcli con up "FRC-Static"

elif [[ -d /etc/netplan ]]; then
    echo -e "${GREEN}Using Netplan${NC}"

    # Create netplan config
    cat > /etc/netplan/99-frc-static.yaml << EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    ${INTERFACE}:
      dhcp4: false
      addresses:
        - ${STATIC_IP}/24
      routes:
        - to: default
          via: ${GATEWAY}
      # No DNS - robot network is isolated
EOF

    chmod 600 /etc/netplan/99-frc-static.yaml
    netplan apply

else
    echo -e "${GREEN}Using /etc/network/interfaces${NC}"

    # Backup existing config
    cp /etc/network/interfaces /etc/network/interfaces.backup.$(date +%s) 2>/dev/null || true

    # Check if interface already configured
    if grep -q "$INTERFACE" /etc/network/interfaces; then
        echo -e "${YELLOW}Interface $INTERFACE already in /etc/network/interfaces${NC}"
        echo "Updating configuration..."
        # Remove old config for this interface
        sed -i "/auto $INTERFACE/,/^$/d" /etc/network/interfaces
    fi

    # Add static IP config
    cat >> /etc/network/interfaces << EOF

# FRC Robot Network - Static IP
auto ${INTERFACE}
iface ${INTERFACE} inet static
    address ${STATIC_IP}
    netmask 255.255.255.0
    gateway ${GATEWAY}
EOF

    # Restart networking
    ifdown "$INTERFACE" 2>/dev/null || true
    ifup "$INTERFACE"
fi

# Wait for network to come up
sleep 2

# Verify IP assignment
CURRENT_IP=$(ip addr show "$INTERFACE" | grep "inet " | awk '{print $2}' | cut -d/ -f1)
echo ""
echo "Current IP: $CURRENT_IP"

if [[ "$CURRENT_IP" == "$STATIC_IP" ]]; then
    echo -e "${GREEN}✓ Static IP configured successfully!${NC}"
else
    echo -e "${YELLOW}Warning: IP may not have been set correctly${NC}"
    echo "Expected: $STATIC_IP, Got: $CURRENT_IP"
fi

# Test connectivity to roboRIO
echo ""
echo "Testing connection to roboRIO ($ROBORIO_IP)..."
if ping -c 2 -W 2 "$ROBORIO_IP" &> /dev/null; then
    echo -e "${GREEN}✓ roboRIO is reachable!${NC}"
else
    echo -e "${YELLOW}roboRIO not reachable yet (may need to power on robot)${NC}"
fi

# Update vision config with correct roboRIO IP
CONFIG_FILE="/opt/frc-vision/config/config.yml"
if [[ -f "$CONFIG_FILE" ]]; then
    echo ""
    echo "Updating vision config with roboRIO IP..."
    sed -i "s|nt_server:.*|nt_server: \"${ROBORIO_IP}\"|" "$CONFIG_FILE"
    echo -e "${GREEN}✓ Config updated${NC}"

    # Restart vision service
    if systemctl is-active --quiet frc_vision; then
        echo "Restarting vision service..."
        systemctl restart frc_vision
        echo -e "${GREEN}✓ Service restarted${NC}"
    fi
fi

echo ""
echo "==========================================="
echo -e "${GREEN}Network configuration complete!${NC}"
echo ""
echo "Next steps:"
echo "1. Connect Orange Pi to robot radio (aux port or switch)"
echo "2. Power on robot"
echo "3. Check: ping $ROBORIO_IP"
echo "4. Check: sudo systemctl status frc_vision"
echo "==========================================="
