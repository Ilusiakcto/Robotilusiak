# OpenArm VR Teleop

VR teleoperation for the OpenArm bimanual robot over CAN FD, using Placo IK.

This system supports **remote teleoperation** where the VR operator and robot are in different locations (e.g., Kenya ↔ Ireland) connected via Tailscale VPN.

## Architecture

```
┌─────────────────┐      WiFi       ┌─────────────────┐    Tailscale    ┌─────────────────┐
│   Quest VR      │ ──────────────► │   Kenya PC      │ ──────────────► │  Ireland Robot  │
│   (Operator)    │   UDP :5006     │   (UDP Relay)   │   UDP :5006     │   (teleop_udp)  │
└─────────────────┘                 └─────────────────┘                 └─────────────────┘
```

## Quick Start

### Option A: Local Setup (VR and Robot on same network)

```bash
# On robot machine
git clone https://github.com/melvine-git/openarm_teleoperation.git
cd openarm_teleoperation
pip install -r openarm_teleop/requirements.txt
sudo bash openarm_teleop/reinit_can.sh
python -m openarm_teleop.teleop_udp --left-can can0 --right-can can1
```

Configure Quest VR app to connect to robot's IP on port 5006.

### Option B: Remote Setup (VR and Robot in different locations)

See [Remote Teleoperation Setup](#remote-teleoperation-setup) below.

---

## Package Overview

- **`kinematics.py`** — forward/inverse kinematics (Placo) for the 7-DOF arm.
- **`can_motor.py` / `waveshare_can.py`** — CAN interface to Damiao motors.
- **`teleop_udp.py`** — receives VR poses via UDP, runs IK, drives arms.
- **`udp_relay.py`** — relays VR data from local network to remote robot via Tailscale.

## Prerequisites

- Linux x86_64 or aarch64 (e.g. a Jetson Orin) with CAN support
- A CAN FD interface to the arm. Either:
  - Two CANable2 USB adapters (gs_usb compatible, FD firmware), **or**
  - A Waveshare USB-CAN-FD-B dual-channel adapter (see arch note below)
- Python 3.10+
- OpenArm bimanual robot with Damiao motors (DM4310, DM4340, DM8009)
- **UDP mode**: Meta Quest with the OpenArm VR APK
- **Adamo mode**: An Adamo API key

## Setup

### 1. Install Python dependencies

```bash
pip3 install -r requirements.txt
```

### 2. Bring up the CAN FD interfaces

**SocketCAN (CANable2 / gs_usb).** Bring the interfaces up with the helper:

```bash
sudo ./reinit_can.sh             # default interfaces: can0 can1
sudo ./reinit_can.sh can2 can3   # Jetson: native mttcan owns can0/can1
```

On x86_64 / modern kernels gs_usb (with CAN FD) is already in-tree, so this
just loads the module and brings the interfaces up — no build. On a Jetson
(whose stock kernel lacks gs_usb FD) it builds the module from Linux 5.19
source the first time. Verify with `ip link show | grep can` — you should see
your interfaces with `mtu 72`. Pass the interface names you brought up to
teleop (`--right-can can0 --left-can can1`).

**Waveshare USB-CAN-FD-B.** No kernel module needed — the bundled
`waveshare/libcontrolcanfd.so` talks to the device directly. Pass an interface
name of `waveshare:0` (channel 0) or `waveshare:1` (channel 1) instead of a
SocketCAN name. Make sure your udev rules grant access (or run with sudo).

> **Arch note:** the bundled `libcontrolcanfd.so` is built for **ARM aarch64**
> (Jetson). On x86_64, replace it with the vendor's x86_64 build of
> `libcontrolcanfd.so`, or just use the SocketCAN path above.

## Running

### UDP Mode (Recommended)

Start the Quest APK and ensure it's sending UDP packets to port 5006.

SocketCAN example:

```bash
python3 -m openarm_teleop.teleop_udp \
  --right-can can0 \
  --left-can can1 \
  --udp-port 5006 \
  --position-scale 1.0 \
  --max-joint-velocity 1.5
```

Waveshare example:

```bash
python3 -m openarm_teleop.teleop_udp \
  --right-can waveshare:0 \
  --left-can waveshare:1
```

### Adamo Mode (Legacy)

Set your Adamo API key:

```bash
export ADAMO_API_KEY=ak_your_key_here
```

Then run:

```bash
python3 -m openarm_teleop.teleop \
  --right-can can0 \
  --left-can can1 \
  --side both \
  --robot-name openarm \
  --position-scale 1.0 \
  --max-joint-velocity 1.5
```

Run the module from the directory that contains the `openarm_teleop/` folder.

### CLI arguments (UDP mode)

| Argument | Default | Description |
|---|---|---|
| `--udp-host` | `0.0.0.0` | UDP host to listen on |
| `--udp-port` | `5006` | UDP port to listen on |
| `--right-can` | — | CAN interface for right arm (`canN` or `waveshare:N`) |
| `--left-can` | — | CAN interface for left arm (`canN` or `waveshare:N`) |
| `--rate` | `100.0` | Control loop frequency (Hz) |
| `--position-scale` | `1.0` | VR position to robot position multiplier |
| `--max-joint-velocity` | `2.0` | Max joint velocity (rad/s) |
| `--motor-kp` | `10.0` | Motor position gain |
| `--motor-kd` | `1.0` | Motor damping gain |

### CLI arguments (Adamo mode)

| Argument | Default | Description |
|---|---|---|
| `--api-key` | `$ADAMO_API_KEY` | Adamo API key |
| `--robot-name` | `openarm` | Robot name for the Adamo topic |
| `--urdf-path` | bundled | Path to the bimanual URDF |
| `--right-can` | `can0` | CAN interface for the right arm (`canN` or `waveshare:N`) |
| `--left-can` | `can1` | CAN interface for the left arm (`canN` or `waveshare:N`) |
| `--side` | `both` | Which arm(s) to control: `left`, `right`, `both` |
| `--position-scale` | `1.0` | VR position to robot position multiplier |
| `--max-joint-velocity` | `6.5` | Max joint velocity (rad/s) — lower = smoother |
| `--control-rate` | `60.0` | Control loop frequency (Hz) |
| `--motor-kp` | `35.0` | Motor position gain |
| `--motor-kd` | `1.0` | Motor damping gain |

Press **R** in the terminal (or the **X** button on the VR controller) to
recalibrate the IK origin at runtime.

### Tuning tips

- **Too violent/jerky**: lower `--max-joint-velocity` (try 1.0–1.5) and/or
  lower `--motor-kp`.
- **Too sluggish**: raise `--position-scale` and `--max-joint-velocity`.
- **Motor damping**: increase `--motor-kd` for more damping at the motor level.

## Quick CAN connectivity test

```bash
python3 -m openarm_teleop.can_motor can2          # SocketCAN
python3 -m openarm_teleop.can_motor waveshare:0   # Waveshare
```

Enables every motor and reads back its state — a fast way to confirm wiring
and bus configuration before running teleop.

## Troubleshooting

### `OSError: [Errno 105] No buffer space available`

CAN buffer is full (usually from lost ACKs). Reset the interfaces:

```bash
sudo ip link set can2 down && sudo ip link set can3 down
sudo ip link set can2 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can3 up type can bitrate 1000000 dbitrate 5000000 fd on
```

### `OSError: [Errno 19] No such device` / `Cannot find device "can2"`

The gs_usb module needs reloading and the interfaces bringing back up — rerun
`sudo ./reinit_can.sh`. If the adapters still aren't seen, unplug and replug
them and check `lsusb | grep -i can`.

### USB bandwidth issues with a camera

If a camera and the CAN adapters share a USB bus you may get dropped frames.
Move the camera to a different USB port (check the topology with `lsusb -t`).

## CAN protocol notes

- **Send**: Classical CAN frames (16-byte, standard format).
- **Receive**: CAN FD frames (72-byte) — the Damiao motors respond in FD.
- Motors use MIT-mode protocol for position/velocity/torque control.

---

## Remote Teleoperation Setup

This section covers setting up teleoperation between two locations (e.g., Kenya and Ireland) using Tailscale VPN.

### Step 1: Install Tailscale (Both Machines)

**On Kenya PC (Windows):**
1. Download from https://tailscale.com/download
2. Install and sign in with your account
3. Note your Tailscale IP: `tailscale ip -4`

**On Ireland Robot (Linux):**
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4  # Note this IP
```

Verify connectivity:
```bash
# From Kenya, ping Ireland
ping <ireland-tailscale-ip>

# From Ireland, ping Kenya
ping <kenya-tailscale-ip>
```

### Step 2: Clone Repository (Ireland Robot)

```bash
# Install git
sudo apt update && sudo apt install -y git

# Clone the repo
git clone https://github.com/melvine-git/openarm_teleoperation.git
cd openarm_teleoperation

# Create virtual environment
sudo apt install -y python3 python3-pip python3-venv
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r openarm_teleop/requirements.txt

# Install CAN utilities
sudo apt install -y can-utils iproute2
```

### Step 3: Setup CAN Interfaces (Ireland Robot)

```bash
# Make script executable and run
chmod +x openarm_teleop/reinit_can.sh
sudo bash openarm_teleop/reinit_can.sh

# Verify interfaces are up
ip link show can0
ip link show can1
```

### Step 4: Configure IP Addresses

**On Kenya PC**, update the relay to point to Ireland's Tailscale IP:

```bash
# Edit udp_relay.py or use command line argument
python -m openarm_teleop.udp_relay --remote-host <ireland-tailscale-ip>
```

**Example IPs:**
| Location | Tailscale IP | Role |
|----------|--------------|------|
| Kenya PC | 100.85.255.15 | UDP Relay |
| Ireland Robot | 100.82.113.60 | Robot Controller |

### Step 5: Configure Quest VR App

1. Find Kenya PC's **local WiFi IP**:
   - Windows: `ipconfig` → look for `192.168.x.x`
   - Linux: `ip addr | grep inet`

2. In Quest VR app settings, set:
   - **Host:** Kenya PC's local IP (e.g., `192.168.1.129`)
   - **Port:** `5006`

### Step 6: Start the Pipeline

**1. Start Kenya UDP Relay (Windows):**
```bash
cd openarm_teleoperation
python -m openarm_teleop.udp_relay --remote-host <ireland-tailscale-ip>
```

Expected output:
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

**2. Start Ireland Teleop (Linux):**
```bash
cd openarm_teleoperation
source .venv/bin/activate
python -m openarm_teleop.teleop_udp \
  --left-can can0 \
  --right-can can1 \
  --max-joint-velocity 1.0
```

**3. Put on Quest VR headset and start the OpenArm VR app**

### Step 7: Verify Connection

On Kenya PC, you should see:
```
[relay] Forwarded 100 packets (49.8 KB, 20.0 pkt/s)
```

On Ireland Robot, you should see VR position data being received.

---

## Teleop Command Reference

### Basic Commands

```bash
# Single arm (left only)
python -m openarm_teleop.teleop_udp --left-can can0

# Single arm (right only)
python -m openarm_teleop.teleop_udp --right-can can1

# Both arms
python -m openarm_teleop.teleop_udp --left-can can0 --right-can can1

# With velocity limit
python -m openarm_teleop.teleop_udp --left-can can0 --right-can can1 --max-joint-velocity 1.0

# With clutch mode (robot only moves while trigger held)
python -m openarm_teleop.teleop_udp --left-can can0 --right-can can1 --clutch

# Mirrored controls (when viewing robot from front)
python -m openarm_teleop.teleop_udp --left-can can0 --right-can can1 --mirror
```

### Recalibration

Press **R** in terminal or **X button** on VR controller to recalibrate IK origin.

---

## Remote Troubleshooting

### Quest not sending data to Kenya relay

1. Check Quest is on same WiFi as Kenya PC
2. Verify Kenya PC local IP: `ipconfig`
3. Check firewall allows UDP port 5006:
   ```powershell
   # Windows - add firewall rule
   netsh advfirewall firewall add rule name="OpenArm UDP" dir=in action=allow protocol=UDP localport=5006
   ```

### Kenya relay not forwarding to Ireland

1. Verify Tailscale is connected: `tailscale status`
2. Ping Ireland: `ping <ireland-tailscale-ip>`
3. Check relay is using correct IP: `--remote-host <ireland-tailscale-ip>`

### Ireland not receiving data

1. Check teleop is listening on port 5006
2. Verify CAN interfaces: `ip link show can0`
3. Check firewall: `sudo ufw allow 5006/udp`

### Robot arm not responding

1. Verify CAN setup: `sudo bash openarm_teleop/reinit_can.sh`
2. Test CAN connectivity: `candump can0`
3. Check correct `--left-can` and `--right-can` arguments

### High latency

1. Check Tailscale connection quality: `tailscale ping <remote-ip>`
2. Lower `--max-joint-velocity` for smoother movement
3. Ensure both machines have stable internet
