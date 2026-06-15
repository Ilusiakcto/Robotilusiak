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

This script runs on the Kenya PC and:
1. Listens for UDP packets from the Quest VR headset on local network
2. Forwards them to the Ireland robot via Tailscale VPN

Usage:
    python udp_relay.py

Quest VR app should connect to: 192.168.1.129:5006 (Kenya PC local IP)
Data is forwarded to: 100.123.116.112:5006 (Ireland Tailscale IP)
"""

import socket
import argparse
import os

# Configuration from environment variables (with defaults)
LOCAL_HOST = os.environ.get("UDP_RELAY_LOCAL_HOST", "0.0.0.0")
LOCAL_PORT = int(os.environ.get("UDP_RELAY_LOCAL_PORT", "5006"))
REMOTE_HOST = os.environ.get("UDP_RELAY_REMOTE_HOST", "100.123.116.112")
REMOTE_PORT = int(os.environ.get("UDP_RELAY_REMOTE_PORT", "5006"))


def relay_udp(local_host, local_port, remote_host, remote_port):
    """Relay UDP packets from local to remote and back."""
    
    # Socket to receive from Quest (local network)
    local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_socket.bind((local_host, local_port))
    
    # Socket to send to Ireland (Tailscale)
    remote_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"[relay] UDP Relay started", flush=True)
    print(f"[relay] Listening on {local_host}:{local_port}", flush=True)
    print(f"[relay] Forwarding to {remote_host}:{remote_port}", flush=True)
    print(f"[relay] Configure Quest VR app to connect to your local IP port {local_port}", flush=True)
    print("-" * 60, flush=True)
    
    quest_addr = None
    packet_count = 0
    
    while True:
        try:
            # Receive data from Quest or Ireland
            data, addr = local_socket.recvfrom(65535)
            
            # Check if this is from the Quest (local network) or Ireland (Tailscale)
            if addr[0].startswith("192.168.") or addr[0].startswith("10.") or addr[0] == "127.0.0.1":
                # From Quest - forward to Ireland
                quest_addr = addr
                remote_socket.sendto(data, (remote_host, remote_port))
                packet_count += 1
                if packet_count % 100 == 1:
                    print(f"[relay] Quest -> Ireland: {len(data)} bytes (total: {packet_count} packets)")
            else:
                # From Ireland - forward back to Quest
                if quest_addr:
                    local_socket.sendto(data, quest_addr)
                    
        except Exception as e:
            print(f"[relay] Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="UDP Relay for VR Teleoperation over Tailscale")
    parser.add_argument("--local-port", type=int, default=LOCAL_PORT, help="Local port to listen on")
    parser.add_argument("--remote-host", type=str, default=REMOTE_HOST, help="Remote Tailscale IP (Ireland)")
    parser.add_argument("--remote-port", type=int, default=REMOTE_PORT, help="Remote port")
    args = parser.parse_args()
    
    try:
        relay_udp(LOCAL_HOST, args.local_port, args.remote_host, args.remote_port)
    except KeyboardInterrupt:
        print("\n[relay] Stopped.")


if __name__ == "__main__":
    main()
