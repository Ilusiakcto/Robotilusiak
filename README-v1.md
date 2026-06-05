# OpenArm v1 VR Teleoperation with MuJoCo

This guide explains how to set up and run VR teleoperation for **OpenArm v1** using MuJoCo simulation with the dora-rs framework.

## Key Differences from v2

| Feature | v1 | v2 |
|---------|----|----|
| Gripper | Linkage-driven parallel gripper | Compact gripper with in-hand camera |
| Wrist cameras | ❌ None | ✅ Built-in |
| MJCF keyframes | ❌ None | ✅ home, etc. |
| Scenes | `openarm_bimanual.xml` only | cell, demo, pedestal |

## Prerequisites

- Python 3.10+
- Meta Quest 3 headset with Developer Hub
- [uv](https://github.com/astral-sh/uv) package manager (recommended)

## VR Setup (One-time)

1. Create a [Meta Quest Developer account](https://developer.oculus.com/) and install Meta Quest Developer Hub
2. Download the teleoperation APK from the OpenArm documentation
3. Sideload the APK onto your Quest 3 via Developer Hub

## Installation

```bash
# Clone the repository
git clone --recurse-submodules https://github.com/enactic/dora-openarm-data-collection.git
cd dora-openarm-data-collection

# Create virtual environment
uv venv .venv

# Activate (Linux/macOS)
source .venv/bin/activate
# Activate (Windows PowerShell)
.\.venv\Scripts\Activate.ps1

# Install dora-rs CLI
uv pip install dora-rs-cli

# Build the v1 dataflow
dora build dataflow-vr-mujoco-v1.yaml --uv
```

## Per-Session VR Setup

1. Put on the Quest headset and launch the teleoperation app
2. Press the **left controller menu button** to open settings
3. Enter your PC's IP address and port (default: `5006`)
4. Verify communication from PC:
   ```bash
   # Linux/macOS
   nc -lu 5006
   
   # Windows (PowerShell)
   # Use a UDP listener tool or check the dora output
   ```
5. Tape the center-of-eye sensor to keep the headset active
6. Hang the headset around your neck and operate with controllers

## Running the Simulation

```bash
# Run the v1 MuJoCo teleoperation
dora run dataflow-vr-mujoco-v1.yaml --uv
```

The MuJoCo viewer will open showing the OpenArm v1 bimanual robot. Move your VR controllers to control the robot arms.

## Data Collection

The data collection web UI opens automatically at http://localhost:8000

### VR Controller Commands

| Button | Action |
|--------|--------|
| **A** (right controller) | Start recording / Mark success |
| **B** (right controller) | Stop recording / Mark failure |

### Workflow

1. Start the dataflow → Web UI opens
2. Press **A** to start recording
3. Perform the task with VR controllers
4. Press **A** to mark success, or **B** to mark failure
5. Data is saved to `vr_mujoco_v1_data/` directory

## Configuration

### Changing VR Port

Edit `dataflow-vr-mujoco-v1.yaml`:

```yaml
- id: udp-receiver
  args: "--host 0.0.0.0 --port YOUR_PORT"
```

### Changing Data Output Directory

Edit `dataflow-vr-mujoco-v1.yaml`:

```yaml
- id: recorder
  env:
    DIRECTORY: "your_custom_directory"
```

### Using a Custom MJCF

You can specify a custom MJCF file:

```yaml
- id: mujoco-v1
  args: "--xml /path/to/your/custom.xml --enable-collision --ctrl --viewer"
```

## Troubleshooting

### No VR data received
- Check that the Quest app shows the correct IP and port
- Ensure no firewall is blocking UDP port 5006
- Try disabling other VR apps that might interfere

### MuJoCo viewer doesn't open
- Ensure you have a display available
- On headless systems, set `DISPLAY=:1` and run Xvfb

### Robot doesn't move smoothly
- Check network latency between Quest and PC
- Reduce IK iterations if needed: `--max-iters 5`

## Files Created

| File | Purpose |
|------|---------|
| `dataflow-vr-mujoco-v1.yaml` | Main dataflow configuration for v1 |
| `metadata_mujoco_v1.yaml` | Metadata for v1 data collection |
| `nodes/dora-openarm-mujoco/src/dora_openarm_mujoco/main_v1.py` | v1 MuJoCo node |

## Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────┐
│ Quest VR    │────▶│ UDP Receiver │────▶│   IK    │
│ Controllers │     └──────────────┘     └────┬────┘
└─────────────┘                               │
                                              ▼
┌─────────────┐     ┌──────────────┐     ┌─────────┐
│  Recorder   │◀────│  MuJoCo v1   │◀────│ Joints  │
└─────────────┘     └──────────────┘     └─────────┘
```

## License

Apache License 2.0 - See LICENSE for details.
