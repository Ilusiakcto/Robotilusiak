# OpenArm V1 VR Teleoperation

VR teleoperation for [OpenArm V1](https://openarm.dev/) using Meta Quest and [dora-rs](https://dora-rs.ai/).

## Prerequisites

- Python 3.10+
- [uv](https://github.com/astral-sh/uv) package manager
- [dora-rs](https://dora-rs.ai/) CLI: `pip install dora-rs`
- Meta Quest headset with OpenArm teleoperation APK sideloaded

---

## Option A: Real Robot (Kenya + Ireland)

Use this when operating the physical robot remotely over Tailscale VPN.

### Setup

**Kenya PC (Operator):**
- Quest VR headset on same WiFi network
- Tailscale installed and connected

**Ireland PC (Robot):**
- Ubuntu Linux with CAN interfaces (`can0`, `can1`)
- Tailscale connected
- Physical OpenArm V1 robot connected

### Step 1: Ireland PC - Setup CAN & Start Dataflow

```bash
# Setup CAN interfaces
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# Start the robot dataflow
cd dora-openarm-data-collection
dora run dataflow-vr-real-v1.yaml --uv
```

### Step 2: Kenya PC - Start UDP Relay

Create a `.env` file from `.env.example` and set `UDP_RELAY_REMOTE_HOST` to Ireland's Tailscale IP.

```bash
cd dora-openarm-data-collection
python udp_relay.py
```

### Step 3: Quest VR - Connect

1. Put on Quest headset
2. Launch the OpenArm teleoperation app
3. Enter Kenya PC's local IP (e.g., `192.168.1.129`) and port `5006`
4. Press triggers (>30%) to start controlling the arms

---

## Option B: MuJoCo Simulation (Local)

Use this to test VR control locally without the real robot.

### Step 1: Start Simulation Dataflow

```bash
cd dora-openarm-data-collection
dora run dataflow-vr-mujoco-v1.yaml --uv
```

### Step 2: Quest VR - Connect

1. Put on Quest headset
2. Launch the OpenArm teleoperation app  
3. Enter your PC's local IP and port `5006`
4. Press triggers (>30%) to control the simulated arms

---

## Project Structure

```
├── config/                       # Robot configuration
│   ├── openarm_v1_left.yaml
│   └── openarm_v1_right.yaml
├── models/v1/                    # MuJoCo models
├── nodes/                        # Dora nodes
│   ├── dora-openarm/             # Motor driver
│   ├── dora-openarm-kinematics/  # IK solver
│   ├── dora-openarm-mujoco/      # MuJoCo simulation
│   └── dora-openarm-vr/          # VR receiver
├── dataflow-vr-real-v1.yaml      # Real robot dataflow
├── dataflow-vr-mujoco-v1.yaml    # Simulation dataflow
├── udp_relay.py                  # UDP relay for VR
└── .env.example                  # Environment config template
```

---

## Troubleshooting

### Motor Faults (Red Blinking LED)
- **Cause**: Torque overload from aggressive position commands
- **Fix**: Kp/Kd gains have been reduced in config files

### Arms Not Moving
1. Ensure triggers are pressed >30%
2. Verify UDP relay is running
3. Check CAN interfaces are up: `ip link show can0`

### CAN Interface Not Found
```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```
