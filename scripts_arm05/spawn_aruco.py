#!/usr/bin/env python3

import argparse
import os
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self, aruco_nr: int, aruco_positions: list):
        super().__init__('spawn_entity')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

        # Number of aruco cubes to spawn (max 5 for now)
        self.aruco_nr = aruco_nr
        self.aruco_positions = aruco_positions
        self.max_rand_x = 1.0
        self.max_rand_y = 1.0

        # Paths for aruco models
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
            self.final_aruco_positions.append((x, y))
            fpath = self.aruco_models_dir / str(i) / self.model_filename
            with fpath.open('r') as file:
                xml_str = file.read()
                self.req.initial_pose.position.x = x
                self.req.initial_pose.position.y = y
                self.req.name = 'aruco_'+str(i)
                self.req.xml = xml_str
                self.future = self.client.call_async(self.req)
            # time.sleep(0.2) # In case of troubles with spawning, uncomment

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
        description="This script spawns aruco cubes in the simulated gazebo world"
    )

    parser.add_argument('-s', '--seed', 
                        type=int,
                        help="The random seed for numpy. It can be used to obtain constant positions of the cubes.")

    # parser.parse_args(['-h'])
    args, _ = parser.parse_known_args()
    
    if args.seed is not None:
        np.random.seed(args.seed)

    minimal_client = MinimalClientAsync(aruco_nr=5,
                                        aruco_positions=aruco_positions)
    
    minimal_client.get_logger().info("Seed: {}".format(args.seed))


    minimal_client.spawn_aruco_cubes()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    '{} Aruco cubes spawned successfully'.format(minimal_client.aruco_nr))
                minimal_client.log_aruco_positions()
            break
        

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
