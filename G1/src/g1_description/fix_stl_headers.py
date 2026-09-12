import os

meshes_dir = '/home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_description/meshes'

fixed_count = 0
for filename in os.listdir(meshes_dir):
    if filename.lower().endswith('.stl'):
        filepath = os.path.join(meshes_dir, filename)
        with open(filepath, 'rb') as f:
            data = f.read()
        
        # Check if the file starts with b'solid'
        if data.startswith(b'solid'):
            # Replace the 5 bytes of 'solid' with 'robot'
            new_data = b'robot' + data[5:]
            with open(filepath, 'wb') as f:
                f.write(new_data)
            print(f"Successfully patched header for: {filename}")
            fixed_count += 1

print(f"Done! Patched {fixed_count} STL files.")
