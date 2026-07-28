# OpenArm V1 VR Teleoperation

VR teleoperation for [OpenArm V1](https://openarm.dev/) using Meta Quest and [dora-rs](https://dora-rs.ai/).

This system supports **remote teleoperation** where the VR operator and robot are in different locations (e.g., Kenya ↔ Ireland) connected via Tailscale VPN.

## Architecture

```
┌─────────────────┐      WiFi       ┌─────────────────┐    Tailscale    ┌─────────────────┐
│   Quest VR      │ ──────────────► │   Kenya PC      │ ──────────────► │  Ireland Robot  │
│   (Operator)    │   UDP :5006     │   (UDP Relay)   │   UDP :5006     │   (teleop_udp)  │
└─────────────────┘                 └─────────────────┘                 └─────────────────┘
```

## Prerequisites

- Python 3.10+
- [uv](https://github.com/astral-sh/uv) package manager
- [dora-rs](https://dora-rs.ai/) CLI: `pip install dora-rs`
- Meta Quest headset with OpenArm teleoperation APK sideloaded
- [Tailscale](https://tailscale.com/) VPN (for remote teleoperation)

---

## Option A: Real Robot (Kenya + Ireland)

Use this when operating the physical robot remotely over Tailscale VPN.

### Step 1: Install Tailscale (Both Machines)

**Kenya PC (Windows):**
1. Download from https://tailscale.com/download
2. Install and sign in with your account
3. Note your Tailscale IP:
   ```powershell
   tailscale ip -4
   ```

**Ireland PC (Linux):**
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4  # Note this IP (e.g., 100.82.113.60)
```

**Verify connectivity:**
```bash
# From Kenya, ping Ireland
ping <ireland-tailscale-ip>
```

### Step 2: Clone Repository (Ireland PC)

```bash
# Install dependencies
sudo apt update && sudo apt install -y git python3 python3-pip python3-venv can-utils

# Clone the repo
git clone https://github.com/melvine-git/openarm_teleoperation.git
cd openarm_teleoperation

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install Python packages
pip install -r openarm_teleop/requirements.txt
```

### Step 3: Setup CAN Interfaces (Ireland PC)

```bash
# Make script executable and run
chmod +x openarm_teleop/reinit_can.sh
sudo bash openarm_teleop/reinit_can.sh

# Verify interfaces are up
ip link show can0
ip link show can1
```

### Step 4: Configure IP Addresses

**Create `.env` file** in the `openarm_teleop` folder:

```bash
cd openarm_teleoperation/openarm_teleop
cp .env.example .env
```

**Edit `.env`** and set your Ireland Tailscale IP:

```bash
# Kenya PC - UDP Relay Configuration
UDP_RELAY_REMOTE_HOST=100.82.113.60  # Ireland's Tailscale IP
UDP_RELAY_LOCAL_PORT=5006

# Network IPs (for reference)
KENYA_LOCAL_IP=192.168.1.129         # Your local WiFi IP
IRELAND_TAILSCALE_IP=100.82.113.60   # Ireland's Tailscale IP
```

**Example IPs:**
| Location | Tailscale IP | Local WiFi IP | Role |
|----------|--------------|---------------|------|
| Kenya PC | 100.85.255.15 | 192.168.1.129 | UDP Relay |
| Ireland PC | 100.82.113.60 | — | Robot Controller |

### Step 5: Start Ireland Teleop

```bash
cd openarm_teleoperation
source .venv/bin/activate

# Run teleop with both arms
python -m openarm_teleop.teleop_udp \
  --left-can can0 \
  --right-can can1 \
  --max-joint-velocity 1.0
```

**CLI Options:**
| Argument | Default | Description |
|----------|---------|-------------|
| `--left-can` | — | CAN interface for left arm (e.g., `can0`) |
| `--right-can` | — | CAN interface for right arm (e.g., `can1`) |
| `--max-joint-velocity` | `2.0` | Max joint speed in rad/s (lower = smoother) |
| `--motor-kp` | `35.0` | Motor position gain |
| `--motor-kd` | `1.0` | Motor damping gain |
| `--mirror` | off | Mirror controls (when viewing robot from front) |
| `--clutch` | off | Robot only moves while trigger is held |

### Step 6: Start Kenya UDP Relay

```bash
cd openarm_teleoperation

# Option A: Use .env file (recommended)
python -m openarm_teleop.udp_relay

# Option B: Override with command line
python -m openarm_teleop.udp_relay --remote-host 100.82.113.60
```

**Expected output:**
```
============================================================
  OpenArm UDP Relay (Kenya -> Ireland via Tailscale)
============================================================
  Local:  0.0.0.0:5006
  Remote: 100.82.113.60:5006
------------------------------------------------------------
  Waiting for Quest VR data...
============================================================
```

### Step 7: Configure Quest VR App

1. Find Kenya PC's **local WiFi IP**:
   - Windows: `ipconfig` → look for `192.168.x.x`
   - Linux: `ip addr | grep inet`

2. In Quest VR app settings, set:
   - **Host:** Kenya PC's local IP (e.g., `192.168.1.129`)
   - **Port:** `5006`

### Step 8: Start Teleoperation

1. Put on Quest headset
2. Launch the OpenArm teleoperation app
3. Press triggers (>30%) to start controlling the arms

**Verify connection** - Kenya PC should show:
```
[relay] Forwarded 100 packets (49.8 KB, 20.0 pkt/s)
```

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
├── dora-openarm-data-collection/
│   ├── config/                       # Robot configuration
│   │   ├── openarm_v1_left.yaml
│   │   └── openarm_v1_right.yaml
│   ├── nodes/                        # Dora nodes
│   │   ├── dora-openarm/             # Motor driver
│   │   ├── dora-openarm-kinematics/  # IK solver
│   │   ├── dora-openarm-mujoco/      # MuJoCo simulation
│   │   └── dora-openarm-vr/          # VR receiver
│   ├── dataflow-vr-real-v1.yaml      # Real robot dataflow
│   ├── dataflow-vr-mujoco-v1.yaml    # Simulation dataflow
│   ├── udp_relay.py                  # UDP relay for VR
│   └── .env.example                  # Environment config template
│
└── openarm_mujoco/                   # MuJoCo models (V1)
    └── v1/
```

---

## Troubleshooting

### Quest Not Sending Data to Kenya Relay

1. Check Quest is on same WiFi as Kenya PC
2. Verify Kenya PC local IP: `ipconfig` (Windows) or `ip addr` (Linux)
3. Check firewall allows UDP port 5006:
   ```powershell
   # Windows - add firewall rule
   netsh advfirewall firewall add rule name="OpenArm UDP" dir=in action=allow protocol=UDP localport=5006
   ```

### Kenya Relay Not Forwarding to Ireland

1. Verify Tailscale is connected: `tailscale status`
2. Ping Ireland: `ping <ireland-tailscale-ip>`
3. Check relay command has correct IP: `--remote-host <ireland-tailscale-ip>`

### Ireland Not Receiving Data

1. Check teleop is running and listening on port 5006
2. Verify CAN interfaces: `ip link show can0`
3. Check firewall: `sudo ufw allow 5006/udp`

### Motor Faults (Red Blinking LED)

- **Cause**: Torque overload from aggressive position commands
- **Fix**: Kp/Kd gains have been reduced in config files

### Arms Not Moving

1. Ensure triggers are pressed >30%
2. Verify UDP relay is running and forwarding packets
3. Check CAN interfaces are up: `ip link show can0`
4. Verify correct arm specified in dataflow config

### CAN Interface Not Found

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

### High Latency

1. Check Tailscale connection quality: `tailscale ping <remote-ip>`
2. Ensure both machines have stable internet
3. Consider using wired ethernet instead of WiFi

---

## Quick Reference

### Kenya PC Commands
```bash
# Find local IP (for Quest VR app)
ipconfig  # Windows

# Setup .env (first time only)
cd openarm_teleoperation/openarm_teleop
cp .env.example .env
# Edit .env and set UDP_RELAY_REMOTE_HOST to Ireland's Tailscale IP

# Start UDP relay
cd openarm_teleoperation
python -m openarm_teleop.udp_relay
```

### Ireland PC Commands
```bash
# Setup CAN
sudo bash openarm_teleop/reinit_can.sh

# Start teleop
cd openarm_teleoperation
source .venv/bin/activate
python -m openarm_teleop.teleop_udp --left-can can0 --right-can can1 --max-joint-velocity 1.0
```

### Quest VR App Settings
- **Host:** Kenya PC's local WiFi IP (e.g., `192.168.1.129`)
- **Port:** `5006`
