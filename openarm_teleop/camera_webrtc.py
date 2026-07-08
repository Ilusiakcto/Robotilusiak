#!/usr/bin/env python3
"""
WebRTC camera streaming server for OpenArm teleoperation.
Runs on Ireland PC - streams USB camera via WebRTC for low-latency viewing.

Usage:
    pip install aiortc aiohttp opencv-python
    python camera_webrtc.py --camera 0 --port 8889

Then open http://<ireland-ip>:8889 in browser from Kenya.
"""

import argparse
import asyncio
import json
import sys
import time
from typing import Optional

try:
    import cv2
    import numpy as np
    from aiohttp import web
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from aiortc.contrib.media import MediaPlayer, MediaRelay
    from av import VideoFrame
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("\nInstall with: pip install aiortc aiohttp opencv-python av")
    sys.exit(1)


class CameraVideoTrack(VideoStreamTrack):
    """Video track that captures from OpenCV camera."""
    
    kind = "video"
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480, fps: int = 30):
        super().__init__()
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap: Optional[cv2.VideoCapture] = None
        self._start_time = None
        self._frame_count = 0
        
    async def recv(self):
        # Initialize camera on first frame
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self._start_time = time.time()
            print(f"Camera {self.camera_id} opened for WebRTC stream")
            
        # Calculate pts for timing
        if self._start_time is None:
            self._start_time = time.time()
            
        # Read frame
        ret, frame = self.cap.read()
        if not ret:
            # Return black frame if camera fails
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            
        # Convert BGR to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = self._frame_count
        video_frame.time_base = fractions.Fraction(1, self.fps)
        self._frame_count += 1
        
        # Pace the frames
        await asyncio.sleep(1 / self.fps)
        
        return video_frame
        
    def stop(self):
        if self.cap:
            self.cap.release()
            self.cap = None


import fractions

# Global state
pcs = set()
camera_track: Optional[CameraVideoTrack] = None
relay: Optional[MediaRelay] = None


async def index(request):
    """Serve the WebRTC viewer page."""
    html = """<!DOCTYPE html>
<html>
<head>
    <title>OpenArm WebRTC Camera</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            background: #0a0a0f; 
            display: flex; 
            flex-direction: column;
            align-items: center; 
            justify-content: center;
            min-height: 100vh;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        }
        h1 { color: #fff; margin-bottom: 20px; font-weight: 300; }
        #video-container {
            position: relative;
            background: #1a1a2e;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 8px 32px rgba(0,0,0,0.4);
        }
        video { 
            display: block;
            max-width: 95vw;
            max-height: 80vh;
        }
        #status {
            position: absolute;
            top: 12px;
            left: 12px;
            display: flex;
            align-items: center;
            gap: 8px;
            color: #fff;
            font-size: 14px;
            background: rgba(0,0,0,0.6);
            padding: 6px 12px;
            border-radius: 20px;
        }
        #status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: #f00;
        }
        #status-dot.connected { background: #0f0; }
        #stats {
            position: absolute;
            top: 12px;
            right: 12px;
            color: #aaa;
            font-size: 12px;
            font-family: monospace;
            background: rgba(0,0,0,0.6);
            padding: 6px 12px;
            border-radius: 8px;
        }
        #controls {
            margin-top: 20px;
            display: flex;
            gap: 10px;
        }
        button {
            background: #4a4a6a;
            color: #fff;
            border: none;
            padding: 10px 24px;
            border-radius: 8px;
            cursor: pointer;
            font-size: 14px;
            transition: background 0.2s;
        }
        button:hover { background: #6a6a8a; }
        button:disabled { background: #333; cursor: not-allowed; }
    </style>
</head>
<body>
    <h1>OpenArm Camera Stream</h1>
    <div id="video-container">
        <video id="video" autoplay playsinline muted></video>
        <div id="status">
            <div id="status-dot"></div>
            <span id="status-text">Connecting...</span>
        </div>
        <div id="stats"></div>
    </div>
    <div id="controls">
        <button id="btn-connect" onclick="connect()">Connect</button>
        <button id="btn-fullscreen" onclick="toggleFullscreen()">Fullscreen</button>
    </div>
    
    <script>
        let pc = null;
        
        async function connect() {
            const statusDot = document.getElementById('status-dot');
            const statusText = document.getElementById('status-text');
            const btnConnect = document.getElementById('btn-connect');
            
            statusText.textContent = 'Connecting...';
            btnConnect.disabled = true;
            
            // Close existing connection
            if (pc) {
                pc.close();
            }
            
            // Create peer connection
            const config = {
                iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
            };
            pc = new RTCPeerConnection(config);
            
            pc.ontrack = (event) => {
                document.getElementById('video').srcObject = event.streams[0];
                statusDot.classList.add('connected');
                statusText.textContent = 'Live';
            };
            
            pc.oniceconnectionstatechange = () => {
                if (pc.iceConnectionState === 'disconnected' || pc.iceConnectionState === 'failed') {
                    statusDot.classList.remove('connected');
                    statusText.textContent = 'Disconnected';
                    btnConnect.disabled = false;
                }
            };
            
            // Add transceiver for receiving video
            pc.addTransceiver('video', { direction: 'recvonly' });
            
            // Create offer
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            
            // Wait for ICE gathering
            await new Promise((resolve) => {
                if (pc.iceGatheringState === 'complete') {
                    resolve();
                } else {
                    pc.onicegatheringstatechange = () => {
                        if (pc.iceGatheringState === 'complete') resolve();
                    };
                }
            });
            
            // Send offer to server
            const response = await fetch('/offer', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    sdp: pc.localDescription.sdp,
                    type: pc.localDescription.type
                })
            });
            
            const answer = await response.json();
            await pc.setRemoteDescription(new RTCSessionDescription(answer));
            
            // Start stats monitoring
            setInterval(updateStats, 1000);
        }
        
        async function updateStats() {
            if (!pc) return;
            const stats = await pc.getStats();
            let statsText = '';
            stats.forEach(report => {
                if (report.type === 'inbound-rtp' && report.kind === 'video') {
                    const fps = report.framesPerSecond || 0;
                    const width = report.frameWidth || 0;
                    const height = report.frameHeight || 0;
                    statsText = `${width}x${height} @ ${fps.toFixed(0)} FPS`;
                }
            });
            document.getElementById('stats').textContent = statsText;
        }
        
        function toggleFullscreen() {
            const video = document.getElementById('video');
            if (document.fullscreenElement) {
                document.exitFullscreen();
            } else {
                video.requestFullscreen();
            }
        }
        
        // Auto-connect on load
        window.onload = connect;
    </script>
</body>
</html>"""
    return web.Response(content_type="text/html", text=html)


async def offer(request):
    """Handle WebRTC offer from client."""
    global relay, camera_track
    
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    pc = RTCPeerConnection()
    pcs.add(pc)
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state: {pc.connectionState}")
        if pc.connectionState == "failed" or pc.connectionState == "closed":
            await pc.close()
            pcs.discard(pc)
    
    # Add video track
    if relay is None:
        relay = MediaRelay()
    
    pc.addTrack(relay.subscribe(camera_track))
    
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })


async def on_shutdown(app):
    """Clean up on shutdown."""
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    if camera_track:
        camera_track.stop()


def list_cameras(max_cameras: int = 10):
    """List available cameras."""
    print("Scanning for cameras...")
    found = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  Camera {i}: {w}x{h}")
            found.append(i)
            cap.release()
    if not found:
        print("  No cameras found")
    return found


def main():
    global camera_track
    
    parser = argparse.ArgumentParser(description="WebRTC camera server for OpenArm")
    parser.add_argument("--list", action="store_true", help="List available cameras")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--width", type=int, default=640, help="Frame width")
    parser.add_argument("--height", type=int, default=480, help="Frame height")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS")
    parser.add_argument("--port", type=int, default=8889, help="HTTP server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Bind address")
    args = parser.parse_args()
    
    if args.list:
        list_cameras()
        return
    
    # Create camera track
    camera_track = CameraVideoTrack(args.camera, args.width, args.height, args.fps)
    
    # Create web app
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    app.on_shutdown.append(on_shutdown)
    
    print(f"\nWebRTC camera server running:")
    print(f"  Open http://<your-ip>:{args.port}/ in browser")
    print(f"  Camera: {args.camera} ({args.width}x{args.height} @ {args.fps} FPS)")
    print("\nPress Ctrl+C to stop\n")
    
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
