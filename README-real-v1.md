# OpenArm V1 Real Robot VR Teleoperation

This guide explains how to set up VR teleoperation for the real OpenArm V1 robot over Tailscale.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           KENYA (Local)                                  │
│  ┌──────────────┐     ┌─────────────────┐                               │
│  │  Quest VR    │────▶│  UDP Relay      │                               │
│  │  Headset     │     │  (udp_relay.py) │                               │
│  │              │     │  192.168.1.129  │                               │
│  └──────────────┘     └────────┬────────┘                               │
│                                │                                         │
└────────────────────────────────┼─────────────────────────────────────────┘
                                 │ Tailscale VPN
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          IRELAND (Remote)                                │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                    Dora Dataflow                                 │    │
│  │  ┌─────────────┐   ┌─────────┐   ┌─────────────────────────┐   │    │
│  │  │ UDP Receiver│──▶│   IK    │──▶│  Follower Arms          │   │    │
│  │  │ :5006       │   │ Solver  │   │  (can0: right, can1: left)│   │    │
│  │  └─────────────┘   └─────────┘   └─────────────────────────┘   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
│  100.123.116.112 (Tailscale IP)                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Prerequisites

### Kenya PC (VR Relay)
- Python 3.10+
- Quest VR headset on same WiFi network
- Tailscale connected

### Ireland Machine (Robot)
- Ubuntu Linux with SocketCAN configured
- CAN interfaces: `can0` (right arm), `can1` (left arm)
- Tailscale connected (IP: 100.123.116.112)
- OpenArm driver installed: `pip install openarm-driver`
- OpenArm MuJoCo models: `pip install openarm-mujoco`

## Setup

### 1. Configure CAN Interfaces (Ireland)

```bash
# Set up CAN interfaces
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# Verify
ip link show can0
ip link show can1
```

### 2. Install Dependencies (Ireland)

```bash
cd dora-openarm-data-collection
python -m venv .venv
source .venv/bin/activate
pip install dora-rs openarm-driver openarm-mujoco

# Build the dataflow
dora build dataflow-vr-real-v1.yaml
```

### 3. Start UDP Relay (Kenya)

```bash
cd dora-openarm-data-collection
python udp_relay.py --remote-host 100.123.116.112 --remote-port 5006
```

### 4. Configure Quest VR App (Kenya)
- Set IP address: `192.168.1.129` (Kenya PC local IP)
- Set port: `5006`

### 5. Run Dataflow (Ireland)

```bash
cd dora-openarm-data-collection
source .venv/bin/activate
dora run dataflow-vr-real-v1.yaml
```

## Configuration Files

### `config/openarm_v1_right.yaml`
Configuration for right arm on `can0`.

### `config/openarm_v1_left.yaml`
Configuration for left arm on `can1`.

## V1-Specific Settings

The V1 pipeline includes several important differences from V2:

1. **Initial Pose**: V1 starts with arms in forward-reaching pose (joint2=-90°, joint4=+90°)
2. **Frame Offset**: VR workspace centered at `[0.4, 0.22, 0.7]`
3. **Gripper Type**: Slide joints (0-0.044m) instead of hinge joints
4. **EE Frames**: Uses body frames (`openarm_right_hand_tcp`) instead of sites

## Safety Notes

- The `--align-trigger gripper` flag ensures the robot only moves when the gripper is engaged
- Start with the robot in a safe position before enabling teleoperation
- Keep emergency stop accessible
- Test with slow movements first

## Troubleshooting

### No VR data arriving
1. Check Tailscale connection: `tailscale status`
2. Verify UDP relay is running on Kenya PC
3. Check firewall allows UDP port 5006

### Robot not moving
1. Verify CAN interfaces are up: `ip link show can0`
2. Check motor power is on
3. Verify gripper trigger is pressed (alignment trigger)

### IK errors
1. Ensure openarm-mujoco is installed with V1 models
2. Check the XML path is correct for your installation

## Files

- `dataflow-vr-real-v1.yaml` - Main dataflow for real robot
- `udp_relay.py` - UDP relay script for Kenya PC
- `config/openarm_v1_right.yaml` - Right arm CAN config
- `config/openarm_v1_left.yaml` - Left arm CAN config
- `metadata_real_v1.yaml` - Metadata for data recording
