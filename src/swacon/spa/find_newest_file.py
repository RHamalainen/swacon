import glob
import datetime
from typing import Dict, Optional
from pathlib import Path


def find_newest_file(string: str, path_folder: Path, print_output: bool = True) -> Optional[Path]:
    assert isinstance(string, str)
    assert isinstance(path_folder, Path)
    assert isinstance(print_output, bool)
    paths = glob.glob(f"{str(path_folder)}/{string}")
    if len(paths) == 0:
        return None
    if print_output:
        print(f"found {len(paths)} matching files")
    path_to_timestamp: Dict[Path, datetime.datetime] = dict()
    for path_str in paths:
        path = Path(path_str)
        timestamp = datetime.datetime.fromisoformat(path.stem)
        path_to_timestamp[path] = timestamp
    timestamp_to_path: Dict[datetime.datetime, Path] = {k: v for (v, k) in path_to_timestamp.items()}
    timestamps = list(path_to_timestamp.values())
    most_recent_timestamp = max(timestamps)
    most_recent_path = timestamp_to_path[most_recent_timestamp]
    return most_recent_path
