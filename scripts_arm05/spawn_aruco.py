#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(
        self,
        aruco_nr: int,
        aruco_positions: list,
        spawn_service: Optional[str],
        spawn_service_type: str,
        world_name: str,
    ):
        super().__init__('spawn_entity')

        # Number of aruco cubes to spawn (max 5 for now)
        self.aruco_nr = aruco_nr
        self.aruco_positions = aruco_positions
        self.max_rand_x = 1.0
        self.max_rand_y = 1.0

        # Paths for aruco models
        share_dir = Path(get_package_share_directory('arm05_sim'))
        self.aruco_models_dir = share_dir / 'models' / 'aruco_markers'
        self.model_filename = 'model.sdf'

        self.spawn_service_override = spawn_service
        self.spawn_service_type = spawn_service_type
        self.world_name = world_name
        self.request_factory: Optional[Callable[[str, str, Pose], object]] = None
        self.spawn_client = self._configure_spawn_client()

        self.final_aruco_positions = []

    def _configure_spawn_client(self):
        preferred_order = (
            ['gazebo', 'ros_gz']
            if self.spawn_service_type == 'auto'
            else [self.spawn_service_type]
        )

        for index, mode in enumerate(preferred_order):
            wait_forever = index == len(preferred_order) - 1
            client = self._try_connect_spawn_service(mode, wait_forever)
            if client is not None:
                self.spawn_mode = mode
                return client

        raise RuntimeError(
            "Unable to locate a compatible spawn service."
        )

    def _try_connect_spawn_service(self, mode: str, wait_forever: bool):
        service_name = self.spawn_service_override
        max_attempts = None if wait_forever else 10

        if mode == 'gazebo':
            try:
                from gazebo_msgs.srv import SpawnEntity as ServiceType
            except ImportError:
                self.get_logger().debug('gazebo_msgs not available, skipping classic Gazebo interface')
                return None
            default_service = '/spawn_entity'

            def request_factory(xml: str, name: str, pose: Pose):
                req = ServiceType.Request()
                req.name = name
                req.xml = xml
                req.initial_pose = pose
                return req

        elif mode == 'ros_gz':
            try:
                from ros_gz_interfaces.srv import SpawnEntity as ServiceType
            except ImportError:
                self.get_logger().debug('ros_gz_interfaces not available, skipping Gazebo Sim interface')
                return None
            default_service = f'/world/{self.world_name}/create'

            def request_factory(xml: str, name: str, pose: Pose):
                req = ServiceType.Request()
                req.entity_factory.name = name
                req.entity_factory.sdf = xml
                req.entity_factory.pose = pose
                req.entity_factory.allow_renaming = False
                req.entity_factory.relative_to = 'world'
                return req

        else:
            raise ValueError(f"Unsupported spawn mode: {mode}")

        service_to_use = service_name or default_service
        client = self.create_client(ServiceType, service_to_use)
        attempts = 0
        has_logged = False
        while not client.wait_for_service(timeout_sec=1.0):
            attempts += 1
            if max_attempts is not None and attempts >= max_attempts:
                self.get_logger().debug(
                    "Spawn service '%s' (%s) not available yet, trying next option",
                    service_to_use,
                    mode,
                )
                return None
            if not has_logged:
                self.get_logger().info(
                    "Waiting for spawn service '%s' (%s)...",
                    service_to_use,
                    mode,
                )
                has_logged = True

        self.get_logger().info(
            "Connected to spawn service '%s' (%s)",
            service_to_use,
            mode,
        )
        self.request_factory = request_factory
        self.spawn_service = service_to_use
        return client

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
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            request = self.request_factory(xml_str, f'aruco_{i}', pose)
            future = self.spawn_client.call_async(request)

            if not rclpy.spin_until_future_complete(self, future, timeout_sec=5.0):
                self.get_logger().error(
                    "Timed out waiting for spawn service '%s'",
                    self.spawn_service,
                )
                continue

            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('Service call failed: %r', exc)
                continue

            success = getattr(response, 'success', True)
            if not success:
                status_message = getattr(response, 'status_message', '')
                if status_message:
                    self.get_logger().error(
                        "Failed to spawn 'aruco_%d': %s",
                        i,
                        status_message,
                    )
                else:
                    self.get_logger().error("Failed to spawn 'aruco_%d'", i)
                continue

            self.get_logger().info(
                "Spawned aruco_%d at (%.2f, %.2f)",
                i,
                x,
                y,
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
    parser.add_argument(
        '--spawn-service',
        default=None,
        help="Override the spawn service name (defaults depend on backend).",
    )
    parser.add_argument(
        '--spawn-service-type',
        choices=['auto', 'gazebo', 'ros_gz'],
        default='auto',
        help="Force a specific spawn service implementation.",
    )
    parser.add_argument(
        '--world',
        default='default',
        help="World name used when targeting Gazebo Sim (ros_gz).",
    )

    args, _ = parser.parse_known_args()
    
    if args.seed is not None:
        np.random.seed(args.seed)

    minimal_client = MinimalClientAsync(
        aruco_nr=5,
        aruco_positions=aruco_positions,
        spawn_service=args.spawn_service,
        spawn_service_type=args.spawn_service_type,
        world_name=args.world,
    )

    minimal_client.get_logger().info("Seed: %s", args.seed)

    minimal_client.spawn_aruco_cubes()
    minimal_client.log_aruco_positions()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
