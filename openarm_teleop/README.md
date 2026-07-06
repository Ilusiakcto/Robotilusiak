# OpenArm VR Teleop

VR teleoperation for the OpenArm bimanual robot over CAN FD, using Placo IK.

Two VR input modes are supported:
- **UDP mode** (`teleop_udp.py`) — receives VR poses directly from the Quest APK via UDP. No external dependencies.
- **Adamo mode** (`teleop.py`) — receives VR poses via the Adamo network (requires `adamo` package).

The package has four parts:

- **`kinematics.py`** — forward/inverse kinematics (Placo) for the 7-DOF arm.
- **`can_motor.py` / `waveshare_can.py`** — the CAN interface to the Damiao
  motors (SocketCAN and Waveshare USB-CAN-FD-B backends).
- **`teleop_udp.py`** — UDP mode: receives VR controller/head poses directly
  from the Quest APK, runs IK, and drives the arms. **Recommended for most users.**
- **`teleop.py`** — Adamo mode: subscribes to VR controller/head poses via the
  Adamo network, runs IK, and drives the arms.

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
