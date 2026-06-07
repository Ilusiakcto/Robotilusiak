"""Merge left and right arm positions into single array for IK sync.

This operator receives position states from both arms and outputs a combined
16-element array (8 joints per arm) that the IK node uses to sync its internal
MuJoCo state with the real robot's position.
"""

import pyarrow as pa
import numpy as np


class Operator:
    def __init__(self):
        self.right_pos = None
        self.left_pos = None
    
    def on_event(self, dora_event, send_output):
        if dora_event["type"] != "INPUT":
            return
        
        eid = dora_event["id"]
        value = dora_event["value"]
        
        # Extract qpos from state struct
        if isinstance(value, pa.StructArray):
            qpos = np.array(value.field("qpos"), dtype=np.float32)
        else:
            qpos = np.array(value, dtype=np.float32)
        
        if eid == "position_right":
            self.right_pos = qpos
        elif eid == "position_left":
            self.left_pos = qpos
        
        # Only output when we have both positions
        if self.right_pos is not None and self.left_pos is not None:
            # Combine into 16-element array: [right[8], left[8]]
            combined = np.concatenate([self.right_pos[:8], self.left_pos[:8]], dtype=np.float32)
            send_output("position", pa.array(combined, type=pa.float32()))
