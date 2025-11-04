#!/usr/bin/env python3

import argparse
import os
import subprocess
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class ArucoSpawner(Node):

    def __init__(
        self,
        aruco_nr: int,
        aruco_positions: list,
    ):
        super().__init__('aruco_spawner')

        self.aruco_nr = aruco_nr
        self.aruco_positions = aruco_positions
        self.max_rand_x = 1.0
        self.max_rand_y = 1.0

        share_dir = Path(get_package_share_directory('arm05_sim'))
        self.aruco_models_dir = share_dir / 'models' / 'aruco_markers'
        self.model_filename = 'model.sdf'

        self.final_aruco_positions = []

    def spawn_aruco_cubes(self):
        for i in range(self.aruco_nr):
            aruco_index = np.random.randint(0, len(self.aruco_positions))
            x, y = self.aruco_positions[aruco_index]
            self.get_logger().info("Aruco "+str(i)+": x= "+str(x).ljust(6)+ "y= "+str(y).ljust(6))
            del self.aruco_positions[aruco_index]
            x += (np.random.random()*2 - 1) * self.max_rand_x
            y += (np.random.random()*2 - 1) * self.max_rand_y
            fpath = self.aruco_models_dir / str(i) / self.model_filename
            with fpath.open('r') as file:
                xml_str = file.read()
            name = f'aruco_{i}'

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'spawn_entity',
                '-entity', name,
                '-file', str(fpath),
                '-x', f'{x}',
                '-y', f'{y}',
                '-z', '0.0',
            ]

            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().error(
                    f"Failed to spawn '{name}': {result.stderr.strip()}"
                )
                continue

            self.get_logger().info(result.stdout.strip())
            self.get_logger().info(
                "Spawned aruco_%d at (%.2f, %.2f)" % (i, x, y)
            )
            self.final_aruco_positions.append((x, y))

    def log_aruco_positions(self, log_path: Path | None = None):
        def log_file(self: 'MinimalClientAsync', filename: Path):
            with filename.open('w') as file:
                for i, pose in enumerate(self.final_aruco_positions):
                    file.write(f'Aruco {i}: ')
                    file.write(f'x: {pose[0]} ')
                    file.write(f'y: {pose[1]}\n')

        if log_path is None:
            log_path = Path(os.path.expanduser('~')) / '.ros' / 'arm05_sim' / 'logs'

        log_path.mkdir(parents=True, exist_ok=True)

        timestamp = self.get_clock().now().seconds_nanoseconds()[0]
        log_file(self, log_path / 'latest.txt')
        log_file(self, log_path / f'{timestamp}.txt')

def main(args=None):
    rclpy.init(args=args)
    aruco_positions = [
        [-4.93, -0.05],
        [3.9, 3.6],
        [7.95, 3.18],
        [11.6, 3.6],
        [16.2, 2.0],
        [16.2, -1.7],
        [12.2, -5.5],
        [4.1, -4.3],
        [11.1, -1.3]
    ]

    parser = argparse.ArgumentParser(
        prog="ArucoSpawner",
        description="This script spawns aruco cubes in the simulated gazebo world",
    )

    parser.add_argument(
        '-s',
        '--seed',
        type=int,
        help=(
            "The random seed for numpy. It can be used to obtain constant positions "
            "of the cubes."
        ),
    )
    args, _ = parser.parse_known_args()
    
    if args.seed is not None:
        np.random.seed(args.seed)

    spawner = ArucoSpawner(
        aruco_nr=5,
        aruco_positions=aruco_positions,
    )

    spawner.get_logger().info("Seed: %s" % args.seed)

    spawner.spawn_aruco_cubes()
    spawner.log_aruco_positions()

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
