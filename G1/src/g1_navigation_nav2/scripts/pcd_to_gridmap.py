#!/usr/bin/env python3
import os
import argparse
import numpy as np

def align_pcd_to_gravity(pcd, target_floor_z=-0.75):
    """
    Tự động tìm mặt sàn bằng RANSAC và xoay bản đồ 3D để mặt sàn phẳng hoàn toàn (song song mặt XY),
    sau đó dịch chuyển tịnh tiến để mặt sàn nằm ở độ cao target_floor_z.
    """
    import open3d as o3d
    
    print("Đang phân tích mặt phẳng sàn (RANSAC)...")
    # Downsample nhanh để tìm mặt phẳng sàn
    pcd_down = pcd.voxel_down_sample(0.05)
    plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=2000)
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    
    # Đảm bảo vector pháp tuyến hướng lên trên (Z dương)
    if normal[2] < 0:
        normal = -normal
        d = -d
        
    # Tính góc nghiêng so với phương thẳng đứng [0, 0, 1]
    target = np.array([0.0, 0.0, 1.0])
    angle = np.arccos(np.dot(normal, target))
    
    # Nếu góc nghiêng đủ lớn (> 0.5 độ), thực hiện xoay
    if angle > np.radians(0.5):
        axis = np.cross(normal, target)
        axis_len = np.linalg.norm(axis)
        if axis_len > 1e-6:
            axis = axis / axis_len
            R = pcd.get_rotation_matrix_from_axis_angle(axis * angle)
            pcd.rotate(R, center=(0, 0, 0))
            print(f"-> Phát hiện bản đồ bị nghiêng {np.degrees(angle):.2f} độ. Đã xoay cân bằng phương ngang.")
            # Xoay cả các điểm inliers để tính toán độ cao sàn sau khi xoay chính xác
            inlier_points = np.asarray(pcd_down.select_by_index(inliers).points) @ R.T
        else:
            inlier_points = np.asarray(pcd_down.select_by_index(inliers).points)
    else:
        print("-> Bản đồ đã phẳng, không cần xoay cân bằng.")
        inlier_points = np.asarray(pcd_down.select_by_index(inliers).points)
        
    # Tính toán độ cao sàn trung bình sau khi xoay
    avg_z = np.mean(inlier_points[:, 2])
    
    # Dịch chuyển tịnh tiến theo trục Z để đưa sàn về đúng cao độ target_floor_z
    translation_z = target_floor_z - avg_z
    pcd.translate((0, 0, translation_z))
    print(f"-> Đã dịch chuyển trục Z thêm {translation_z:.3f}m để đưa mặt sàn về cao độ chuẩn {target_floor_z}m.")
    return pcd

def convert_pcd_to_gridmap(pcd_path, pgm_path, yaml_path, resolution=0.05, min_z=-0.55, max_z=0.45, align_gravity=True):
    print(f"Đang đọc file PCD: {pcd_path}")
    if not os.path.exists(pcd_path):
        print(f"Lỗi: File {pcd_path} không tồn tại.")
        return False

    try:
        import open3d as o3d
    except ImportError:
        print("Error: Thư viện open3d chưa được cài đặt. Hãy chạy 'pip install open3d'.")
        return False

    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # Tự động phát hiện nếu file đã được căn chỉnh hoặc lọc từ trước
    base_name = os.path.basename(pcd_path)
    if "aligned" in base_name or "filter" in base_name or "clean" in base_name:
        if align_gravity:
            print("-> Phát hiện file đã được căn chỉnh (aligned/filter/clean). Tự động tắt căn chỉnh trọng lực (no-align).")
            align_gravity = False

    if align_gravity:
        # Tự động cân bằng trọng lực và căn chỉnh chiều cao sàn
        pcd = align_pcd_to_gravity(pcd, target_floor_z=-0.75)
        
        # Lưu file PCD đã căn chỉnh trọng lực ra một file riêng
        dir_name = os.path.dirname(pcd_path)
        name, ext = os.path.splitext(base_name)
        if not name.endswith("_aligned"):
            aligned_pcd_path = os.path.join(dir_name, f"{name}_aligned{ext}")
            o3d.io.write_point_cloud(aligned_pcd_path, pcd)
            print(f"Đã lưu bản đồ 3D sạch đã căn chỉnh vào: {aligned_pcd_path}")

    points = np.asarray(pcd.points)
    print(f"Đang xử lý {len(points)} điểm mây để chiếu thành 2D Gridmap...")

    # Lọc các điểm nằm trong khoảng chiều cao (waist/body height)
    z_mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    filtered_points = points[z_mask]
    print(f"-> Giữ lại {len(filtered_points)} điểm nằm trong khoảng Z [{min_z:.2f}m, {max_z:.2f}m].")

    if len(filtered_points) == 0:
        print("Lỗi: Không có điểm nào nằm trong khoảng lọc chiều cao!")
        return False

    xs = filtered_points[:, 0]
    ys = filtered_points[:, 1]

    min_x, max_x = np.min(xs), np.max(xs)
    min_y, max_y = np.min(ys), np.max(ys)

    # Thêm lề biên xung quanh bản đồ
    margin = 1.0
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin

    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    print(f"Kích thước lưới Grid: {width} x {height} ô. Điểm gốc bản đồ (Origin): [{min_x:.3f}, {min_y:.3f}, 0.0]")

    # Khởi tạo bản đồ lưới: 254 (free/trống), 0 (occupied/vật cản)
    grid = np.full((height, width), 254, dtype=np.uint8)

    # Điền các điểm vật cản vào lưới và giãn nở nhẹ (dilation) để tránh khe hở tường
    for x, y, _ in filtered_points:
        col = int((x - min_x) / resolution)
        row = int((y - min_y) / resolution)
        if 0 <= col < width and 0 <= row < height:
            grid[row, col] = 0
            # Giãn nở 3x3 xung quanh điểm vật cản
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    r, c = row + dr, col + dc
                    if 0 <= r < height and 0 <= c < width:
                        grid[r, c] = 0

    # Lưu ảnh PGM (phải lật ngược dòng để khớp định dạng ROS)
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        for r in reversed(range(height)):
            f.write(grid[r, :].tobytes())
    print(f"Đã lưu ảnh bản đồ 2D PGM vào: {pgm_path}")

    # Tạo nội dung file cấu hình YAML đi kèm
    yaml_content = f"""image: {os.path.basename(pgm_path)}
resolution: {resolution}
origin: [{min_x:.6f}, {min_y:.6f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"Đã lưu file cấu hình YAML vào: {yaml_path}")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert PCD pointcloud to ROS2 2D Occupancy Grid map with auto gravity alignment.")
    parser.add_name = parser.add_argument
    parser.add_name("--pcd", default="/home/hoangdc/ROS2/unitree_G1/maps/g1_office/g1_office_aligned_filter.pcd", help="Input PCD path")
    parser.add_name("--pgm", default="/home/hoangdc/ROS2/unitree_G1/maps/g1_office/g1_office.pgm", help="Output PGM path")
    parser.add_name("--yaml", default="/home/hoangdc/ROS2/unitree_G1/maps/g1_office/g1_office.yaml", help="Output YAML path")
    parser.add_name("--resolution", type=float, default=0.05, help="Map resolution in meters/cell")
    parser.add_name("--min_z", type=float, default=-0.6, help="Min height threshold (rel to aligned floor Z=-0.75m)")
    parser.add_name("--max_z", type=float, default=0.45, help="Max height threshold (rel to aligned floor Z=-0.75m)")
    parser.add_name("--no-align", action="store_true", help="Disable automatic gravity alignment")
    args = parser.parse_args()

    convert_pcd_to_gridmap(
        pcd_path=args.pcd,
        pgm_path=args.pgm,
        yaml_path=args.yaml,
        resolution=args.resolution,
        min_z=args.min_z,
        max_z=args.max_z,
        align_gravity=not args.no_align
    )