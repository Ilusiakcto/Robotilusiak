#!/usr/bin/env python3
"""
Camera streaming client for OpenArm teleoperation.
Runs on Kenya PC - receives and displays MJPEG stream from Ireland.

Usage:
    python camera_client.py --url http://ireland-ip:8888/stream
    python camera_client.py --url http://ireland-ip:8888/stream --fullscreen
"""

import argparse
import sys
import time
import threading
from typing import Optional
from urllib.request import urlopen
from urllib.error import URLError

try:
    import cv2
    import numpy as np
except ImportError:
    print("Error: OpenCV not installed. Run: pip install opencv-python")
    sys.exit(1)


class MJPEGClient:
    """MJPEG stream client with reconnection support."""
    
    def __init__(self, url: str):
        self.url = url
        self.frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.connected = False
        self.fps = 0.0
        self.latency_ms = 0.0
        
    def start(self):
        """Start stream receiver thread."""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        
    def _receive_loop(self):
        """Continuously receive frames with auto-reconnect."""
        while self.running:
            try:
                self._connect_and_receive()
            except Exception as e:
                self.connected = False
                if self.running:
                    print(f"Connection error: {e}. Reconnecting in 2s...")
                    time.sleep(2)
                    
    def _connect_and_receive(self):
        """Connect to stream and receive frames."""
        print(f"Connecting to {self.url}...")
        
        stream = urlopen(self.url, timeout=10)
        self.connected = True
        print("Connected!")
        
        buffer = b''
        frame_count = 0
        fps_start = time.time()
        
        while self.running:
            # Read chunk
            chunk = stream.read(4096)
            if not chunk:
                raise ConnectionError("Stream ended")
            buffer += chunk
            
            # Find JPEG boundaries
            start = buffer.find(b'\xff\xd8')  # JPEG start
            end = buffer.find(b'\xff\xd9')    # JPEG end
            
            if start != -1 and end != -1 and end > start:
                # Extract JPEG
                jpeg_data = buffer[start:end+2]
                buffer = buffer[end+2:]
                
                # Decode frame
                frame_start = time.time()
                nparr = np.frombuffer(jpeg_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    with self.frame_lock:
                        self.frame = frame
                        
                    # Calculate FPS
                    frame_count += 1
                    elapsed = time.time() - fps_start
                    if elapsed >= 1.0:
                        self.fps = frame_count / elapsed
                        frame_count = 0
                        fps_start = time.time()
                        
    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest frame."""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
            
    def stop(self):
        """Stop receiver."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(description="Camera streaming client for OpenArm")
    parser.add_argument("--url", type=str, required=True,
                        help="MJPEG stream URL (e.g., http://ireland-ip:8888/stream)")
    parser.add_argument("--fullscreen", action="store_true",
                        help="Start in fullscreen mode")
    parser.add_argument("--window-width", type=int, default=960,
                        help="Window width (default: 960)")
    parser.add_argument("--window-height", type=int, default=720,
                        help="Window height (default: 720)")
    args = parser.parse_args()
    
    # Create client
    client = MJPEGClient(args.url)
    client.start()
    
    # Create window
    window_name = "OpenArm Camera - Press Q to quit, F for fullscreen"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, args.window_width, args.window_height)
    
    if args.fullscreen:
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    
    fullscreen = args.fullscreen
    
    print("\nControls:")
    print("  Q - Quit")
    print("  F - Toggle fullscreen")
    print("  S - Save snapshot")
    print()
    
    try:
        while True:
            frame = client.get_frame()
            
            if frame is not None:
                # Add overlay info
                h, w = frame.shape[:2]
                status = "LIVE" if client.connected else "RECONNECTING..."
                color = (0, 255, 0) if client.connected else (0, 0, 255)
                
                # Status indicator
                cv2.circle(frame, (20, 25), 8, color, -1)
                cv2.putText(frame, status, (35, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, color, 2)
                
                # FPS
                cv2.putText(frame, f"{client.fps:.1f} FPS", (w - 100, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                cv2.imshow(window_name, frame)
            else:
                # Show waiting screen
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                text = "Waiting for stream..." if not client.connected else "Receiving..."
                cv2.putText(blank, text, (150, 240), cv2.FONT_HERSHEY_SIMPLEX,
                           1, (128, 128, 128), 2)
                cv2.imshow(window_name, blank)
                
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # Q or ESC
                break
            elif key == ord('f'):
                fullscreen = not fullscreen
                if fullscreen:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, 
                                         cv2.WINDOW_FULLSCREEN)
                else:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, 
                                         cv2.WINDOW_NORMAL)
            elif key == ord('s'):
                if frame is not None:
                    filename = f"snapshot_{int(time.time())}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Saved: {filename}")
                    
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...")
        client.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
