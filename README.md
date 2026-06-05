# OpenArm V1 VR Teleoperation

VR teleoperation for [OpenArm V1](https://openarm.dev/) using Meta Quest and [dora-rs](https://dora-rs.ai/).

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              KENYA (Operator)                                │
│                                                                              │
│   ┌──────────────┐      ┌─────────────────┐                                 │
│   │  Quest VR    │─UDP─▶│   UDP Relay     │                                 │
│   │  Headset     │      │  udp_relay.py   │                                 │
│   │  (WiFi)      │      │  :5006          │                                 │
│   └──────────────┘      └────────┬────────┘                                 │
│                                  │                                           │
└──────────────────────────────────┼───────────────────────────────────────────┘
                                   │ Tailscale VPN
                                   ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                             IRELAND (Robot)                                  │
│                                                                              │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                        Dora Dataflow                                 │   │
│   │                                                                      │   │
│   │  ┌─────────────┐   ┌─────────┐   ┌───────────────────────────────┐  │   │
│   │  │ UDP Receiver│──▶│   IK    │──▶│     Follower Arms             │  │   │
│   │  │ :5006       │   │ Solver  │   │  can0: right  /  can1: left   │  │   │
│   │  └─────────────┘   └─────────┘   └───────────────────────────────┘  │   │
│   │                                                                      │   │
│   │  Safety Features:                                                    │   │
│   │  • V1 Joint Limits (J2: ±100°)                                      │   │
│   │  • Velocity Limiting (3-6 rad/s)                                    │   │
│   │  • Acceleration Limiting (10-20 rad/s²)                             │   │
│   │  • Quest Input Smoothing (1€ Filter)                                │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│   Tailscale IP: 100.123.116.112                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Quick Start

### Option A: MuJoCo Simulation (Local Testing)

Test the full pipeline locally without the real robot:

```bash
# On your local machine
cd dora-openarm-data-collection
dora build dataflow-vr-mujoco-v1.yaml --uv
dora run dataflow-vr-mujoco-v1.yaml --uv
```

Configure Quest VR app to connect to your PC's local IP on port `5006`.

### Option B: Real Robot (Kenya → Ireland)

#### Step 1: Kenya PC (VR Relay)

```bash
# Start the UDP relay to forward Quest data to Ireland
cd dora-openarm-data-collection
python udp_relay.py --remote-host 100.123.116.112 --remote-port 5006
```

Configure Quest VR app:
- IP: Your Kenya PC's local IP (e.g., `192.168.1.129`)
- Port: `5006`

#### Step 2: Ireland Machine (Robot)

```bash
# 1. Set up CAN interfaces
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# 2. Pull latest code
cd openarm_teleoperation
git pull origin main

# 3. Build and run
dora build dataflow-vr-real-v1.yaml --uv
dora run dataflow-vr-real-v1.yaml --uv
```

## V1-Specific Safety Features

OpenArm V1 has different joint limits than V2. This pipeline includes:

| Feature | Description | Config |
|---------|-------------|--------|
| **Joint Limits** | J2: ±100° (V2 is -190° to +10°) | `config/openarm_sim.yaml` |
| **Velocity Limits** | 3-6 rad/s per joint | `config/openarm_sim.yaml` |
| **Acceleration Limits** | 10-20 rad/s² per joint | `config/openarm_sim.yaml` |
| **Input Smoothing** | 1€ Filter on Quest poses | Built-in |
| **Safety Clamp** | Enforces limits before motor commands | Built-in |

## Configuration Files

| File | Description |
|------|-------------|
| `dataflow-vr-mujoco-v1.yaml` | MuJoCo simulation dataflow |
| `dataflow-vr-real-v1.yaml` | Real robot dataflow |
| `config/openarm_sim.yaml` | Joint limits, velocity/accel limits, offsets |
| `config/openarm_v1_right.yaml` | Right arm CAN config (can0) |
| `config/openarm_v1_left.yaml` | Left arm CAN config (can1) |
| `udp_relay.py` | UDP relay for Kenya → Ireland |

## Troubleshooting

### No VR data arriving (Ireland)
1. Check Tailscale: `tailscale status`
2. Verify UDP relay running on Kenya
3. Check firewall allows UDP port 5006: `sudo ufw allow 5006/udp`

### Robot not moving
1. Verify CAN interfaces: `ip link show can0`
2. Check motor power is on
3. **Squeeze gripper trigger first** (alignment safety)

### IK errors
1. Ensure `openarm-mujoco` is installed: `pip install openarm-mujoco`
2. Check Quest is sending valid poses

### Joint clamping messages
```
[safety] Joint 2 clamped: 120.0 -> 100.0 deg
```
This is normal - the safety system is preventing V2-range commands from reaching V1 motors.

## Prerequisites

### Kenya PC
- Python 3.10+
- Meta Quest on same WiFi
- Tailscale connected

### Ireland Machine
- Ubuntu Linux with SocketCAN
- CAN interfaces: `can0` (right), `can1` (left)
- Tailscale connected
- `pip install dora-rs openarm-driver openarm-mujoco`

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.
