#!/usr/bin/env python3
"""Record both live ROS odometry topics for the comparison plot."""

import argparse
import csv
import json
from pathlib import Path
import threading
import time

import rospy
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    names = ("original", "highrate")
    paths = {name: args.output_dir / (name + ".csv") for name in names}
    if any(path.exists() for path in paths.values()):
        parser.error("Output CSVs already exist; choose a new --output-dir")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Start the recorder before playing the bag. Both topics use the same clock.
    rospy.init_node("record_highrate_odometry", anonymous=True)
    lock = threading.Lock()
    streams, writers = {}, {}
    counts = {name: 0 for name in names}
    frames = {name: set() for name in names}
    closed = False
    subscribers = []
    try:
        for name, path in paths.items():
            streams[name] = path.open("x", newline="")
            writers[name] = csv.writer(streams[name])
            writers[name].writerow(("stamp", "x", "y", "z", "arrival_monotonic"))
            streams[name].flush()

        def receive(message, name):
            arrival = time.monotonic()
            position = message.pose.pose.position
            with lock:
                if closed:
                    return
                writers[name].writerow((
                    message.header.stamp.to_sec(), position.x, position.y,
                    position.z, arrival,
                ))
                counts[name] += 1
                frames[name].add(message.header.frame_id)

        subscribers = [
            rospy.Subscriber("/Odometry", Odometry, receive,
                             callback_args="original", queue_size=10000),
            rospy.Subscriber("/Odometry_highrate", Odometry, receive,
                             callback_args="highrate", queue_size=10000),
        ]
        print(f"Recording to {args.output_dir.resolve()}", flush=True)
        print("Now start your replay. When it finishes, press Ctrl+C here to save.", flush=True)
        # Wall time continues even when rosbag's simulated clock pauses or ends.
        while not rospy.is_shutdown():
            time.sleep(2)
            with lock:
                for stream in streams.values():
                    stream.flush()
                print(f"Received: /Odometry={counts['original']}, "
                      f"/Odometry_highrate={counts['highrate']}", flush=True)
    except KeyboardInterrupt:
        rospy.signal_shutdown("recording stopped")
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            closed = True
            for stream in streams.values():
                stream.close()
            metadata = {
                "topics": {"original": "/Odometry", "highrate": "/Odometry_highrate"},
                "counts": counts,
                "frames": {name: sorted(values) for name, values in frames.items()},
                "positions": "unmodified live message pose values",
                "arrival_clock": "shared process monotonic clock, seconds",
            }
        (args.output_dir / "recording.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Saved {paths['original']} and {paths['highrate']}", flush=True)
        if any(count == 0 for count in counts.values()):
            print("One or both topics produced no messages; check the replay and high-rate switch.")


if __name__ == "__main__":
    main()
