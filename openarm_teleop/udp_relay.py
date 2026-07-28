#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
UDP Relay Script for VR Teleoperation over Tailscale
=====================================================

This script runs on the Kenya PC and:
1. Listens for UDP packets from the Quest VR headset on local network
2. Forwards them to the Ireland robot via Tailscale VPN

Architecture:
    [Quest VR] --UDP--> [Kenya PC (this relay)] --Tailscale--> [Ireland Robot]
                              192.168.x.x:5006                   100.x.x.x:5006

Usage:
    # On Kenya PC
    python -m openarm_teleop.udp_relay --remote-host 100.123.116.112

    # On Ireland Robot
    python -m openarm_teleop.teleop_udp --right-can can0 --left-can can1

Quest VR app should connect to: <Kenya PC local IP>:5006
Data is forwarded to: <Ireland Tailscale IP>:5006
"""

import socket
import argparse
import select
import os
import time
from pathlib import Path

# Load .env file if it exists
def load_dotenv():
    """Load environment variables from .env file."""
    env_paths = [
        Path(__file__).parent / ".env",  # openarm_teleop/.env
        Path.cwd() / ".env",              # current directory
    ]
    for env_path in env_paths:
        if env_path.exists():
            with open(env_path) as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        key, value = line.split("=", 1)
                        os.environ.setdefault(key.strip(), value.strip())
            break

load_dotenv()

# Configuration from environment variables (with defaults)
LOCAL_HOST = os.environ.get("UDP_RELAY_LOCAL_HOST", "0.0.0.0")
LOCAL_PORT = int(os.environ.get("UDP_RELAY_LOCAL_PORT", "5006"))
REMOTE_HOST = os.environ.get("UDP_RELAY_REMOTE_HOST", "100.82.113.60")
REMOTE_PORT = int(os.environ.get("UDP_RELAY_REMOTE_PORT", "5006"))


def relay_udp(local_host: str, local_port: int, remote_host: str, remote_port: int) -> None:
    """
    Relay UDP packets from local Quest VR to remote Ireland robot via Tailscale.
    
    Args:
        local_host: Local interface to bind (0.0.0.0 for all)
        local_port: Local port to listen on
        remote_host: Remote Tailscale IP (Ireland robot)
        remote_port: Remote port on Ireland robot
    """
    # Socket to receive from Quest (local network)
    local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    local_socket.bind((local_host, local_port))
    local_socket.setblocking(False)
    
    print("=" * 60, flush=True)
    print("  OpenArm UDP Relay (Kenya -> Ireland via Tailscale)", flush=True)
    print("=" * 60, flush=True)
    print(f"  Local:  {local_host}:{local_port}", flush=True)
    print(f"  Remote: {remote_host}:{remote_port}", flush=True)
    print("-" * 60, flush=True)
    print("  Configure Quest VR app to connect to your Kenya PC's", flush=True)
    print(f"  local IP address on port {local_port}", flush=True)
    print("-" * 60, flush=True)
    print("  Waiting for Quest VR data...", flush=True)
    print("=" * 60, flush=True)
    
    quest_addr = None
    packet_count = 0
    bytes_forwarded = 0
    start_time = time.monotonic()
    last_status = 0.0
    
    while True:
        try:
            # Use select to wait for data with timeout
            ready, _, _ = select.select([local_socket], [], [], 1.0)
            
            if not ready:
                # Periodic status update
                now = time.monotonic()
                if now - last_status > 10.0 and packet_count > 0:
                    elapsed = now - start_time
                    rate = packet_count / elapsed if elapsed > 0 else 0
                    print(f"[relay] Stats: {packet_count} packets, "
                          f"{bytes_forwarded/1024:.1f} KB, "
                          f"{rate:.1f} pkt/s", flush=True)
                    last_status = now
                continue
            
            # Receive data
            data, addr = local_socket.recvfrom(65535)
            
            # Determine if from Quest (local network) or Ireland (Tailscale)
            is_local = (
                addr[0].startswith("192.168.") or
                addr[0].startswith("10.") or
                addr[0].startswith("172.") or
                addr[0] == "127.0.0.1"
            )
            
            if is_local:
                # From Quest VR - forward to Ireland
                if quest_addr is None or quest_addr != addr:
                    quest_addr = addr
                    print(f"[relay] Quest connected from {addr[0]}:{addr[1]}", flush=True)
                
                # Forward to Ireland via Tailscale
                local_socket.sendto(data, (remote_host, remote_port))
                packet_count += 1
                bytes_forwarded += len(data)
                
                # Log periodically
                if packet_count == 1:
                    print(f"[relay] First packet forwarded to Ireland", flush=True)
                elif packet_count % 500 == 0:
                    elapsed = time.monotonic() - start_time
                    rate = packet_count / elapsed if elapsed > 0 else 0
                    print(f"[relay] Forwarded {packet_count} packets "
                          f"({bytes_forwarded/1024:.1f} KB, {rate:.1f} pkt/s)", flush=True)
            else:
                # From Ireland - forward back to Quest
                if quest_addr:
                    local_socket.sendto(data, quest_addr)
                    
        except BlockingIOError:
            continue
        except Exception as e:
            print(f"[relay] Error: {e}", flush=True)
            time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="UDP Relay for VR Teleoperation (Kenya -> Ireland via Tailscale)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic usage with default Ireland Tailscale IP
    python -m openarm_teleop.udp_relay

    # Specify Ireland Tailscale IP
    python -m openarm_teleop.udp_relay --remote-host 100.123.116.112

    # Custom ports
    python -m openarm_teleop.udp_relay --local-port 5006 --remote-port 5006

Environment variables:
    UDP_RELAY_LOCAL_HOST   Local bind address (default: 0.0.0.0)
    UDP_RELAY_LOCAL_PORT   Local port (default: 5006)
    UDP_RELAY_REMOTE_HOST  Ireland Tailscale IP
    UDP_RELAY_REMOTE_PORT  Ireland port (default: 5006)
"""
    )
    parser.add_argument(
        "--local-host", type=str, default=LOCAL_HOST,
        help="Local host to bind (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--local-port", type=int, default=LOCAL_PORT,
        help="Local port to listen on (default: 5006)"
    )
    parser.add_argument(
        "--remote-host", type=str, default=REMOTE_HOST,
        help="Remote Tailscale IP - Ireland robot (default: 100.123.116.112)"
    )
    parser.add_argument(
        "--remote-port", type=int, default=REMOTE_PORT,
        help="Remote port on Ireland robot (default: 5006)"
    )
    args = parser.parse_args()
    
    try:
        relay_udp(args.local_host, args.local_port, args.remote_host, args.remote_port)
    except KeyboardInterrupt:
        print("\n[relay] Stopped.", flush=True)


if __name__ == "__main__":
    main()
