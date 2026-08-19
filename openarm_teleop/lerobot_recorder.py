"""
LeRobot Dataset Recorder for OpenArm VR Teleoperation

Records joint positions during VR teleoperation and saves in LeRobot v2.1 format.
No camera support - joints only.

Usage:
    recorder = LeRobotRecorder(
        repo_id="username/dataset_name",
        task="pick up the cup",
        fps=30
    )
    
    # In teleop loop:
    recorder.start_episode()
    while teleoperating:
        recorder.add_frame(
            observation_state=current_joints,  # [14] for bimanual
            action=target_joints               # [14] for bimanual
        )
    recorder.end_episode()
    
    # When done:
    recorder.save_and_upload(hf_token="hf_xxx")
"""

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
    PYARROW_AVAILABLE = True
except ImportError:
    PYARROW_AVAILABLE = False
    print("Warning: pyarrow not installed. Install with: pip install pyarrow")

try:
    from huggingface_hub import HfApi
    HF_HUB_AVAILABLE = True
except ImportError:
    HF_HUB_AVAILABLE = False
    print("Warning: huggingface_hub not installed. Install with: pip install huggingface_hub")


@dataclass
class EpisodeData:
    """Data for a single episode."""
    episode_index: int
    frames: List[Dict] = field(default_factory=list)
    task: str = ""
    start_time: float = 0.0


class LeRobotRecorder:
    """
    Records teleoperation data in LeRobot v2.1 format.
    
    Dataset structure:
        dataset_name/
        ├── data/
        │   └── chunk-000/
        │       ├── episode_000000.parquet
        │       └── ...
        ├── meta/
        │   ├── info.json
        │   ├── episodes.jsonl
        │   └── tasks.jsonl
        └── README.md
    """
    
    def __init__(
        self,
        repo_id: str,
        task: str = "teleoperation task",
        fps: int = 30,
        output_dir: str = "./recordings",
        robot_type: str = "openarm",
        state_dim: int = 14,  # 7 joints per arm * 2 arms
        action_dim: int = 14,
    ):
        """
        Initialize the recorder.
        
        Args:
            repo_id: HuggingFace repo ID (e.g., "username/my_dataset")
            task: Task description for the recording session
            fps: Recording framerate
            output_dir: Local directory to save recordings
            robot_type: Robot type identifier
            state_dim: Dimension of observation state (14 for bimanual)
            action_dim: Dimension of action (14 for bimanual)
        """
        if not PYARROW_AVAILABLE:
            raise ImportError("pyarrow is required. Install with: pip install pyarrow")
        
        self.repo_id = repo_id
        self.task = task
        self.fps = fps
        self.robot_type = robot_type
        self.state_dim = state_dim
        self.action_dim = action_dim
        
        # Parse repo_id
        if "/" in repo_id:
            self.dataset_name = repo_id.split("/")[-1]
        else:
            self.dataset_name = repo_id
        
        # Setup paths
        self.output_dir = Path(output_dir) / self.dataset_name
        self.data_dir = self.output_dir / "data" / "chunk-000"
        self.meta_dir = self.output_dir / "meta"
        
        # Create directories
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        
        # State
        self.episodes: List[EpisodeData] = []
        self.current_episode: Optional[EpisodeData] = None
        self.total_frames = 0
        self.is_recording = False
        
        # Task tracking
        self.tasks: Dict[str, int] = {}  # task -> task_index
        self._add_task(task)
        
        print(f"[LeRobotRecorder] Initialized for {repo_id}")
        print(f"  Output: {self.output_dir}")
        print(f"  Task: {task}")
        print(f"  FPS: {fps}")
    
    def _add_task(self, task: str) -> int:
        """Add a task and return its index."""
        if task not in self.tasks:
            self.tasks[task] = len(self.tasks)
        return self.tasks[task]
    
    def start_episode(self, task: Optional[str] = None) -> None:
        """Start recording a new episode."""
        if self.current_episode is not None:
            print("[LeRobotRecorder] Warning: Previous episode not ended. Ending it now.")
            self.end_episode()
        
        episode_task = task or self.task
        task_index = self._add_task(episode_task)
        
        self.current_episode = EpisodeData(
            episode_index=len(self.episodes),
            task=episode_task,
            start_time=time.time(),
        )
        self.is_recording = True
        
        print(f"[LeRobotRecorder] Started episode {self.current_episode.episode_index}")
    
    def add_frame(
        self,
        observation_state: np.ndarray,
        action: np.ndarray,
        timestamp: Optional[float] = None,
    ) -> None:
        """
        Add a frame to the current episode.
        
        Args:
            observation_state: Current joint positions [state_dim]
            action: Target/commanded joint positions [action_dim]
            timestamp: Optional timestamp (seconds from episode start)
        """
        if self.current_episode is None:
            return
        
        if timestamp is None:
            timestamp = time.time() - self.current_episode.start_time
        
        frame_index = len(self.current_episode.frames)
        
        frame = {
            "observation.state": observation_state.tolist(),
            "action": action.tolist(),
            "timestamp": timestamp,
            "episode_index": self.current_episode.episode_index,
            "frame_index": frame_index,
            "index": self.total_frames,
            "task_index": self.tasks[self.current_episode.task],
        }
        
        self.current_episode.frames.append(frame)
        self.total_frames += 1
    
    def end_episode(self, discard: bool = False) -> Optional[int]:
        """
        End the current episode.
        
        Args:
            discard: If True, discard this episode without saving
            
        Returns:
            Episode index if saved, None if discarded
        """
        if self.current_episode is None:
            return None
        
        self.is_recording = False
        
        if discard or len(self.current_episode.frames) == 0:
            print(f"[LeRobotRecorder] Discarded episode {self.current_episode.episode_index}")
            self.total_frames -= len(self.current_episode.frames)
            self.current_episode = None
            return None
        
        # Mark last frame as done
        if self.current_episode.frames:
            self.current_episode.frames[-1]["next.done"] = True
            for frame in self.current_episode.frames[:-1]:
                frame["next.done"] = False
        
        # Save episode to parquet
        self._save_episode_parquet(self.current_episode)
        
        episode_index = self.current_episode.episode_index
        self.episodes.append(self.current_episode)
        self.current_episode = None
        
        print(f"[LeRobotRecorder] Saved episode {episode_index} ({len(self.episodes[-1].frames)} frames)")
        
        return episode_index
    
    def _save_episode_parquet(self, episode: EpisodeData) -> None:
        """Save episode data to parquet file."""
        if not episode.frames:
            return
        
        # Prepare columns
        data = {
            "observation.state": [f["observation.state"] for f in episode.frames],
            "action": [f["action"] for f in episode.frames],
            "timestamp": [f["timestamp"] for f in episode.frames],
            "episode_index": [f["episode_index"] for f in episode.frames],
            "frame_index": [f["frame_index"] for f in episode.frames],
            "index": [f["index"] for f in episode.frames],
            "task_index": [f["task_index"] for f in episode.frames],
            "next.done": [f.get("next.done", False) for f in episode.frames],
        }
        
        # Create table
        table = pa.table({
            "observation.state": pa.array(data["observation.state"], type=pa.list_(pa.float32())),
            "action": pa.array(data["action"], type=pa.list_(pa.float32())),
            "timestamp": pa.array(data["timestamp"], type=pa.float64()),
            "episode_index": pa.array(data["episode_index"], type=pa.int64()),
            "frame_index": pa.array(data["frame_index"], type=pa.int64()),
            "index": pa.array(data["index"], type=pa.int64()),
            "task_index": pa.array(data["task_index"], type=pa.int64()),
            "next.done": pa.array(data["next.done"], type=pa.bool_()),
        })
        
        # Save
        episode_file = self.data_dir / f"episode_{episode.episode_index:06d}.parquet"
        pq.write_table(table, episode_file)
    
    def _save_metadata(self) -> None:
        """Save dataset metadata files."""
        # info.json
        info = {
            "codebase_version": "v2.1",
            "robot_type": self.robot_type,
            "fps": self.fps,
            "total_episodes": len(self.episodes),
            "total_frames": self.total_frames,
            "total_tasks": len(self.tasks),
            "total_videos": 0,
            "total_chunks": 1,
            "chunks_size": 1000,
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "splits": {"train": f"0:{len(self.episodes)}"},
            "features": {
                "observation.state": {
                    "dtype": "float32",
                    "shape": [self.state_dim],
                    "names": [f"joint_{i}" for i in range(self.state_dim)],
                },
                "action": {
                    "dtype": "float32",
                    "shape": [self.action_dim],
                    "names": [f"joint_{i}" for i in range(self.action_dim)],
                },
                "timestamp": {"dtype": "float64", "shape": [1]},
                "episode_index": {"dtype": "int64", "shape": [1]},
                "frame_index": {"dtype": "int64", "shape": [1]},
                "index": {"dtype": "int64", "shape": [1]},
                "task_index": {"dtype": "int64", "shape": [1]},
                "next.done": {"dtype": "bool", "shape": [1]},
            },
        }
        
        with open(self.meta_dir / "info.json", "w") as f:
            json.dump(info, f, indent=2)
        
        # episodes.jsonl
        with open(self.meta_dir / "episodes.jsonl", "w") as f:
            for ep in self.episodes:
                ep_meta = {
                    "episode_index": ep.episode_index,
                    "tasks": [ep.task],
                    "length": len(ep.frames),
                }
                f.write(json.dumps(ep_meta) + "\n")
        
        # tasks.jsonl
        with open(self.meta_dir / "tasks.jsonl", "w") as f:
            for task, task_index in self.tasks.items():
                task_meta = {
                    "task_index": task_index,
                    "task": task,
                }
                f.write(json.dumps(task_meta) + "\n")
        
        # README.md
        readme = f"""---
license: apache-2.0
task_categories:
  - robotics
tags:
  - LeRobot
  - OpenArm
  - teleoperation
---

# {self.dataset_name}

This dataset was recorded using OpenArm VR teleoperation.

- **Robot:** {self.robot_type}
- **Episodes:** {len(self.episodes)}
- **Total Frames:** {self.total_frames}
- **FPS:** {self.fps}
- **Task:** {self.task}

## Dataset Structure

- `observation.state`: Joint positions [{self.state_dim}]
- `action`: Target joint positions [{self.action_dim}]

## Usage

```python
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset("{self.repo_id}")
```
"""
        with open(self.output_dir / "README.md", "w") as f:
            f.write(readme)
        
        print(f"[LeRobotRecorder] Saved metadata to {self.meta_dir}")
    
    def save(self) -> Path:
        """Save dataset locally without uploading."""
        if self.current_episode is not None:
            self.end_episode()
        
        self._save_metadata()
        
        print(f"[LeRobotRecorder] Dataset saved to {self.output_dir}")
        print(f"  Episodes: {len(self.episodes)}")
        print(f"  Total frames: {self.total_frames}")
        
        return self.output_dir
    
    def upload(self, hf_token: str) -> str:
        """
        Upload dataset to HuggingFace Hub.
        
        Args:
            hf_token: HuggingFace token with write permissions
            
        Returns:
            URL of the uploaded dataset
        """
        if not HF_HUB_AVAILABLE:
            raise ImportError("huggingface_hub is required. Install with: pip install huggingface_hub")
        
        # Save metadata first
        self.save()
        
        api = HfApi(token=hf_token)
        
        # Create repo if it doesn't exist
        try:
            api.create_repo(
                repo_id=self.repo_id,
                repo_type="dataset",
                exist_ok=True,
            )
        except Exception as e:
            print(f"[LeRobotRecorder] Repo creation note: {e}")
        
        # Upload folder
        print(f"[LeRobotRecorder] Uploading to {self.repo_id}...")
        api.upload_folder(
            folder_path=str(self.output_dir),
            repo_id=self.repo_id,
            repo_type="dataset",
        )
        
        url = f"https://huggingface.co/datasets/{self.repo_id}"
        print(f"[LeRobotRecorder] Upload complete: {url}")
        
        return url
    
    def save_and_upload(self, hf_token: str) -> str:
        """Save locally and upload to HuggingFace."""
        return self.upload(hf_token)
    
    @property
    def num_episodes(self) -> int:
        """Number of completed episodes."""
        return len(self.episodes)
    
    @property
    def num_frames(self) -> int:
        """Total number of frames across all episodes."""
        return self.total_frames


# Simple test
if __name__ == "__main__":
    # Test recording
    recorder = LeRobotRecorder(
        repo_id="test/openarm_test",
        task="test task",
        fps=30,
        state_dim=14,
        action_dim=14,
    )
    
    # Simulate 2 episodes
    for ep in range(2):
        recorder.start_episode()
        for i in range(30):  # 1 second at 30fps
            state = np.random.randn(14).astype(np.float32)
            action = np.random.randn(14).astype(np.float32)
            recorder.add_frame(state, action)
            time.sleep(0.01)
        recorder.end_episode()
    
    # Save locally
    recorder.save()
    print(f"Test complete: {recorder.num_episodes} episodes, {recorder.num_frames} frames")
