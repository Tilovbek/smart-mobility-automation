#!/bin/bash
# Test script for maintenance monitor node
# Publishes fake battery data, diagnostics, and cmd_vel to test the maintenance monitor

echo "========================================="
echo "Maintenance Monitor Test Script"
echo "========================================="
echo ""
echo "This script will publish mock data to test the maintenance node:"
echo "  - Battery state (gradually draining)"
echo "  - Diagnostic messages"
echo "  - Robot velocity commands"
echo ""
echo "Make sure the maintenance node is running:"
echo "  ros2 launch turtlebot3_automation maintenance.launch.py"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="
echo ""

# Source ROS 2 environment
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Function to publish battery state
publish_battery() {
    local voltage=$1
    local percentage=$2
    ros2 topic pub --once /battery_state sensor_msgs/BatteryState \
        "{voltage: $voltage, percentage: $percentage, power_supply_status: 1, present: true}"
}

# Function to publish diagnostics (simplified - just log)
publish_diagnostics() {
    local level=$1
    local message=$2
    local voltage=$3
    echo "📊 Diagnostics: $message (Battery: ${voltage}V)"
}

# Function to publish cmd_vel
publish_cmd_vel() {
    local linear_x=$1
    local angular_z=$2
    ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
        "{
            linear: {x: $linear_x, y: 0.0, z: 0.0},
            angular: {x: 0.0, y: 0.0, z: $angular_z}
        }"
}

echo "Starting mock data publication..."
echo "Battery will drain from 12.4V to 10.5V over time"
echo ""

# Initial healthy state
publish_battery 12.4 100.0
publish_diagnostics 0 "System healthy" "12.4"
publish_cmd_vel 0.0 0.0
sleep 2

# Simulate battery drain with various scenarios
cycle=1
while true; do
    # Calculate battery values
    voltage=$(echo "scale=1; 12.4 - ($cycle - 1) * 0.19" | bc)
    percentage=$(echo "scale=1; 100 - ($cycle - 1) * 9" | bc)

    echo "Cycle $cycle: Publishing battery ${voltage}V (${percentage}%)"

    publish_battery $voltage $percentage

    # Send some movement commands
    publish_cmd_vel 0.1 0.0
    sleep 1

    # Send diagnostics based on battery level
    if (( $(echo "$voltage < 11.0" | bc -l) )); then
        publish_diagnostics 1 "Low battery warning" "$voltage"
        echo "⚠️  Warning: Low battery!"
    elif (( $(echo "$voltage < 10.8" | bc -l) )); then
        publish_diagnostics 2 "Critical battery level" "$voltage"
        echo "🚨 Critical: Battery critically low!"
    else
        publish_diagnostics 0 "System operating normally" "$voltage"
    fi

    # Reset after 10 cycles
    if [ $cycle -eq 10 ]; then
        echo "Battery critically low - resetting to full charge"
        cycle=0
        sleep 2
    fi

    cycle=$((cycle + 1))
    sleep 3
done
