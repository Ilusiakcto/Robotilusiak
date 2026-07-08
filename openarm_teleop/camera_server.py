#!/usr/bin/env python3
"""
Camera streaming server for OpenArm teleoperation.
Runs on Ireland PC - detects USB camera and streams MJPEG over HTTP.

Usage:
    python camera_server.py --list          # List available cameras
    python camera_server.py --camera 0      # Stream camera 0
    python camera_server.py --port 8888     # Use custom port
"""

import argparse
import sys
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional

try:
    import cv2
    import numpy as np
except ImportError:
    print("Error: OpenCV not installed. Run: pip install opencv-python")
    sys.exit(1)


class CameraCapture:
    """Thread-safe camera capture with frame buffering."""
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480, fps: int = 30):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap: Optional[cv2.VideoCapture] = None
        self.frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
    def start(self) -> bool:
        """Start camera capture thread."""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"Error: Cannot open camera {self.camera_id}")
            return False
            
        # Set resolution and FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Get actual settings
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Camera {self.camera_id}: {actual_w}x{actual_h} @ {actual_fps:.1f} FPS")
        
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        return True
        
    def _capture_loop(self):
        """Continuously capture frames."""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.frame = frame
            else:
                time.sleep(0.01)
                
    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest frame."""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
            
    def get_jpeg(self, quality: int = 80) -> Optional[bytes]:
        """Get latest frame as JPEG bytes."""
        frame = self.get_frame()
        if frame is None:
            return None
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
        return jpeg.tobytes() if ret else None
        
    def stop(self):
        """Stop capture."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()


# Global camera instance
camera: Optional[CameraCapture] = None


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG streaming."""
    
    def log_message(self, format, *args):
        # Suppress default logging
        pass
        
    def do_GET(self):
        global camera
        
        if self.path == '/':
            # Serve simple HTML page with embedded stream
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = b'''<!DOCTYPE html>
<html>
<head>
    <title>OpenArm Camera</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        html, body { width: 100%; height: 100%; background: #000; overflow: hidden; }
        body { display: flex; align-items: center; justify-content: center; }
        img { width: 100vw; height: 100vh; object-fit: contain; cursor: pointer; }
        #controls { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); display: flex; gap: 10px; opacity: 0.7; z-index: 100; }
        #controls:hover { opacity: 1; }
        button { background: #333; color: #fff; border: none; padding: 10px 20px; border-radius: 8px; cursor: pointer; font-size: 14px; }
        button:hover { background: #555; }
        .info { position: fixed; top: 10px; left: 10px; color: #0f0; font-family: monospace; font-size: 14px; z-index: 100; }
    </style>
</head>
<body>
    <div class="info">OpenArm Camera - Press F for fullscreen</div>
    <img id="video" src="/stream" onclick="toggleFullscreen()" />
    <div id="controls">
        <button onclick="toggleFullscreen()">Fullscreen (F)</button>
    </div>
    <script>
        function toggleFullscreen() {
            if (document.fullscreenElement) {
                document.exitFullscreen();
            } else {
                document.documentElement.requestFullscreen();
            }
        }
        document.addEventListener('keydown', (e) => {
            if (e.key === 'f' || e.key === 'F') toggleFullscreen();
        });
    </script>
</body>
</html>'''
            self.wfile.write(html)
            
        elif self.path == '/stream':
            # MJPEG stream
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            
            try:
                while True:
                    jpeg = camera.get_jpeg(quality=75) if camera else None
                    if jpeg:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpeg)}\r\n'.encode())
                        self.wfile.write(b'\r\n')
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    time.sleep(1/30)  # ~30 FPS
            except (BrokenPipeError, ConnectionResetError):
                pass  # Client disconnected
                
        elif self.path == '/snapshot':
            # Single JPEG image
            jpeg = camera.get_jpeg(quality=90) if camera else None
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_error(503, 'Camera not available')
                
        else:
            self.send_error(404, 'Not found')


def list_cameras(max_cameras: int = 10):
    """List available cameras."""
    print("Scanning for cameras...")
    found = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            backend = cap.getBackendName()
            print(f"  Camera {i}: {w}x{h} @ {fps:.0f} FPS ({backend})")
            found.append(i)
            cap.release()
    if not found:
        print("  No cameras found")
    return found


def main():
    global camera
    
    parser = argparse.ArgumentParser(description="Camera streaming server for OpenArm")
    parser.add_argument("--list", action="store_true", help="List available cameras")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--width", type=int, default=640, help="Frame width")
    parser.add_argument("--height", type=int, default=480, help="Frame height")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS")
    parser.add_argument("--port", type=int, default=8888, help="HTTP server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Bind address")
    args = parser.parse_args()
    
    if args.list:
        list_cameras()
        return
        
    # Start camera
    camera = CameraCapture(args.camera, args.width, args.height, args.fps)
    if not camera.start():
        sys.exit(1)
        
    # Start HTTP server
    server = HTTPServer((args.host, args.port), MJPEGHandler)
    print(f"\nCamera server running:")
    print(f"  Local:   http://127.0.0.1:{args.port}/")
    print(f"  Stream:  http://<your-ip>:{args.port}/stream")
    print(f"  Snapshot: http://<your-ip>:{args.port}/snapshot")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        camera.stop()
        server.shutdown()


if __name__ == "__main__":
    main()
