#!/usr/bin/env python3
import os
import sys
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from g1_state_estimator.srv import SaveMap

try:
    from voxblox_msgs.srv import FilePath
except ImportError:
    # Fallback/mock if voxblox_msgs is not fully registered in python path yet
    FilePath = None

class MapSaverClient(Node):
    def __init__(self):
        super().__init__('map_saver_client')

    def call_save_pcd_service(self, dest_dir, prefix):
        self.get_logger().info('Calling /state_estimator/save_map service...')
        client = self.create_client(SaveMap, '/state_estimator/save_map')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/state_estimator/save_map service not available!')
            return False

        req = SaveMap.Request()
        req.destination_path = dest_dir
        req.file_prefix = prefix
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res is not None:
            if res.success:
                self.get_logger().info(f'PCD Map saved successfully: {res.message}')
                return True
            else:
                self.get_logger().error(f'Failed to save PCD Map: {res.message}')
        else:
            self.get_logger().error('Service call to save PCD Map failed.')
        return False

    def call_set_voxblox_parameter(self, mesh_path):
        self.get_logger().info('Updating /voxblox_global parameter "mesh_filename"...')
        client = self.create_client(SetParameters, '/voxblox_global/set_parameters')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/voxblox_global/set_parameters service not available. Skipping dynamic parameter update.')
            return False

        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mesh_path)
        param = Parameter(name='mesh_filename', value=val)
        req.parameters = [param]
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res is not None and len(res.results) > 0:
            if res.results[0].successful:
                self.get_logger().info('Parameter "mesh_filename" updated successfully.')
                return True
            else:
                self.get_logger().warn(f'Failed to set parameter: {res.results[0].reason}')
        else:
            self.get_logger().warn('Failed to call parameter update service.')
        return False

    def call_generate_mesh_service(self):
        self.get_logger().info('Calling /voxblox_global/generate_mesh service...')
        client = self.create_client(Empty, '/voxblox_global/generate_mesh')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/voxblox_global/generate_mesh service not available!')
            return False

        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res is not None:
            self.get_logger().info('Mesh generated and saved successfully.')
            return True
        else:
            self.get_logger().error('Service call to generate mesh failed.')
        return False

    def call_save_voxblox_map(self, map_path):
        if FilePath is None:
            self.get_logger().warn('voxblox_msgs.srv.FilePath could not be imported in Python. Skipping binary .voxblox save.')
            return False

        self.get_logger().info('Calling /voxblox_global/save_map service...')
        client = self.create_client(FilePath, '/voxblox_global/save_map')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/voxblox_global/save_map service not available. Skipping binary save.')
            return False

        req = FilePath.Request()
        req.file_path = map_path
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res is not None:
            self.get_logger().info(f'Binary .voxblox map saved successfully to: {map_path}')
            return True
        else:
            self.get_logger().error('Service call to save binary map failed.')
        return False

def main():
    # 1. Load default configurations from yaml
    script_dir = os.path.dirname(os.path.realpath(__file__))
    config_path = os.path.join(script_dir, '../config/map_saver.yaml')
    
    yaml_config = {}
    if os.path.exists(config_path):
        try:
            import yaml
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f).get('map_saver', {})
        except Exception as e:
            print(f"Warning: Failed to load config from {config_path}: {e}")

    default_dir = yaml_config.get('save_dir', '/home/hoangdc/ROS2/unitree_G1/maps/')
    default_prefix = yaml_config.get('prefix', 'g1_office')
    default_script = yaml_config.get('pcd_to_gridmap_script', '/home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_navigation_nav2/pcd_to_gridmap.py')

    parser = argparse.ArgumentParser(description="Save all maps (PCD, PLY Mesh, and Voxblox binary) simultaneously.")
    parser.add_argument('-d', '--dir', type=str, default=default_dir,
                        help="Target directory to save maps")
    parser.add_argument('-p', '--prefix', type=str, default=default_prefix,
                        help="Prefix filename for all maps")
    
    args, unknown = parser.parse_known_args()
    
    # Tự động tạo thư mục con mang tên map (prefix) bên trong thư mục maps chung
    dest_dir = os.path.join(os.path.abspath(args.dir), args.prefix)
    prefix = args.prefix
    
    pcd_filename = f"{prefix}.pcd"
    mesh_filename = f"{prefix}.ply"
    voxblox_filename = f"{prefix}.voxblox"
    
    mesh_filepath = os.path.join(dest_dir, mesh_filename)
    voxblox_filepath = os.path.join(dest_dir, voxblox_filename)
    
    print("=" * 60)
    print("      Unified Map Saver Tool for Unitree G1 SLAM Stack      ")
    print("=" * 60)
    print(f"Target Directory : {dest_dir}")
    print(f"File Prefix      : {prefix}")
    print(f"Outputs to Save  :")
    print(f"  - 3D PCD Map   : {prefix}.pcd (+ trajectory)")
    print(f"  - 3D Mesh PLY  : {prefix}.ply")
    print(f"  - TSDF Binary  : {prefix}.voxblox")
    print("-" * 60)
    
    # Ensure directory exists
    os.makedirs(dest_dir, exist_ok=True)
    
    rclpy.init()
    saver = MapSaverClient()
    
    # 1. Save PCD map
    pcd_success = saver.call_save_pcd_service(dest_dir, prefix)
    
    # 2. Update mesh path parameter dynamically
    saver.call_set_voxblox_parameter(mesh_filepath)
    
    # 3. Call generate mesh service
    mesh_success = saver.call_generate_mesh_service()
    
    # 4. Save binary voxblox map
    voxblox_success = saver.call_save_voxblox_map(voxblox_filepath)
    
    # 5. Generate 2D Gridmap from saved PCD (with automatic gravity alignment)
    gridmap_success = False
    if pcd_success:
        pcd_filepath = os.path.join(dest_dir, pcd_filename)
        pgm_filepath = os.path.join(dest_dir, f"{prefix}.pgm")
        yaml_filepath = os.path.join(dest_dir, f"{prefix}.yaml")
        
        script_path = default_script
        if os.path.exists(script_path):
            print("-" * 60)
            print("Generating 2D Occupancy Gridmap from PCD...")
            try:
                import subprocess
                cmd = [
                    "python3", script_path,
                    "--pcd", pcd_filepath,
                    "--pgm", pgm_filepath,
                    "--yaml", yaml_filepath
                ]
                # Pass gridmap configs from yaml if defined
                gridmap_config = yaml_config.get('gridmap', {})
                if 'min_z' in gridmap_config:
                    cmd += ["--min_z", str(gridmap_config['min_z'])]
                if 'max_z' in gridmap_config:
                    cmd += ["--max_z", str(gridmap_config['max_z'])]
                if 'resolution' in gridmap_config:
                    cmd += ["--resolution", str(gridmap_config['resolution'])]
                if gridmap_config.get('align_gravity') is False:
                    cmd.append("--no-align")
                    
                res = subprocess.run(cmd, capture_output=True, text=True)
                print(res.stdout)
                if res.returncode == 0:
                    gridmap_success = True
                else:
                    print(f"Error output: {res.stderr}")
            except Exception as e:
                print(f"Failed to execute pcd_to_gridmap.py: {e}")
        else:
            print(f"\nWarning: pcd_to_gridmap.py not found at {script_path}. Skipping 2D Gridmap generation.")
            
    print("-" * 60)
    print("Summary of Map Saving:")
    print(f"  - PCD Map (3D)      : {'SUCCESS' if pcd_success else 'FAILED'}")
    print(f"  - Mesh (PLY 3D)     : {'SUCCESS' if mesh_success else 'FAILED'}")
    print(f"  - TSDF Map (Voxblox): {'SUCCESS' if voxblox_success else 'FAILED'}")
    print(f"  - Gridmap (2D PGM)  : {'SUCCESS' if gridmap_success else 'FAILED'}")
    print("=" * 60)
    
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
