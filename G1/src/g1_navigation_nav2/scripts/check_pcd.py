#!/usr/bin/env python3
import os
import sys
import numpy as np

# Tự động nạp đường dẫn map mặc định
DEFAULT_PCD_PATH = "/home/hoangdc/ROS2/unitree_G1/maps/g1_office_aligned.pcd"

def clean_by_picked_points(pcd_path, h1=0.2):
    print(f"Đang đọc file PCD: {pcd_path}")
    if not os.path.exists(pcd_path):
        print(f"Lỗi: Không tìm thấy file {pcd_path}")
        return

    try:
        import open3d as o3d
    except ImportError:
        print("\n[LỖI] Chưa cài đặt thư viện 'open3d'.")
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        print("Mây điểm rỗng!")
        return

    print("\n=======================================================")
    print("CHẾ ĐỘ CHỌN VÙNG TRỰC QUAN TỪ TRÊN XUỐNG VÀ LỌC ĐỘ CAO")
    print("-------------------------------------------------------")
    print(" HƯỚNG DẪN THAO TÁC:")
    print("  1. Dùng chuột xoay bản đồ nhìn thẳng từ TRÊN xuống dưới (Top-down view).")
    print("  2. Nhấp giữ phím 'Shift' + Click chuột trái vào các vị trí góc")
    print("     để khoanh vùng khu vực có người đi bộ cần xóa (Chọn tối thiểu 2 điểm).")
    print("     Mỗi điểm chọn sẽ hiện một khối cầu nhỏ kèm số thứ tự.")
    print("  3. Sau khi chọn xong vùng bao quanh, nhấn phím 'Q' hoặc 'ESC' để đóng.")
    print("  4. Terminal sẽ tự tính toán biên 2D của vùng bạn chọn và lọc độ cao Z >= h1.")
    print("=======================================================\n")

    # Mở cửa sổ chọn điểm
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Top-down View - Shift+Click để chọn vùng", width=1024, height=768)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    # Lấy các điểm người dùng đã pick
    picked_indices = vis.get_picked_points()
    if len(picked_indices) < 2:
        print("\n[LỖI] Bạn phải chọn tối thiểu 2 điểm (ví dụ 2 góc đối diện của vùng cần xóa).")
        print("Hủy bỏ thao tác.")
        return

    picked_coords = points[picked_indices]
    min_x = picked_coords[:, 0].min()
    max_x = picked_coords[:, 0].max()
    min_y = picked_coords[:, 1].min()
    max_y = picked_coords[:, 1].max()

    print(f"\nĐã xác định vùng cần dọn dẹp từ các điểm bạn chọn:")
    print(f"  * Giới hạn X: [{min_x:.3f}, {max_x:.3f}]")
    print(f"  * Giới hạn Y: [{min_y:.3f}, {max_y:.3f}]")

    # Hỏi nhập chiều cao h1 bắt đầu xóa
    try:
        h1_input = input(f"Nhập độ cao h1 bắt đầu xóa (Z >= h1, mặc định {h1}m): ")
        if h1_input.strip() != "":
            h1 = float(h1_input)
    except ValueError:
        print("Định dạng số không hợp lệ, sử dụng mặc định 0.2m.")

    # Lọc điểm
    in_2d_box = (points[:, 0] >= min_x) & (points[:, 0] <= max_x) & \
                (points[:, 1] >= min_y) & (points[:, 1] <= max_y)
    above_h1 = points[:, 2] >= h1
    
    to_delete = in_2d_box & above_h1
    to_keep_indices = np.where(~to_delete)[0]
    
    pcd_cleaned = pcd.select_by_index(to_keep_indices)
    deleted_count = len(points) - len(pcd_cleaned.points)
    
    print(f"\n====================== KẾT QUẢ DỌN DẸP ======================")
    print(f"  * Tổng số điểm ban đầu: {len(points)}")
    print(f"  * Số điểm bị xóa trong vùng chọn (Z >= {h1}m): {deleted_count}")
    print(f"  * Số điểm còn lại: {len(pcd_cleaned.points)}")
    
    ans = input("Bạn có muốn lưu đè vào file PCD gốc không? (y/n): ")
    if ans.lower() == 'y':
        o3d.io.write_point_cloud(pcd_path, pcd_cleaned)
        print(f"Đã lưu đè thành công vào: {pcd_path}")
    else:
        dir_name = os.path.dirname(pcd_path)
        base_name = os.path.basename(pcd_path)
        name, ext = os.path.splitext(base_name)
        new_path = os.path.join(dir_name, f"{name}_clean_select{ext}")
        o3d.io.write_point_cloud(new_path, pcd_cleaned)
        print(f"Đã lưu thành file mới tại: {new_path}")
    print("=============================================================\n")


def clean_region_by_height(pcd_path, min_x, max_x, min_y, max_y, h1):
    print(f"Đang đọc file PCD: {pcd_path}")
    if not os.path.exists(pcd_path):
        print(f"Lỗi: Không tìm thấy file {pcd_path}")
        return

    try:
        import open3d as o3d
    except ImportError:
        print("\n[LỖI] Chưa cài đặt thư viện 'open3d'.")
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        print("Mây điểm rỗng!")
        return

    in_2d_box = (points[:, 0] >= min_x) & (points[:, 0] <= max_x) & \
                (points[:, 1] >= min_y) & (points[:, 1] <= max_y)
    above_h1 = points[:, 2] >= h1
    
    to_delete = in_2d_box & above_h1
    to_keep_indices = np.where(~to_delete)[0]
    
    pcd_cleaned = pcd.select_by_index(to_keep_indices)
    deleted_count = len(points) - len(pcd_cleaned.points)
    
    print(f"\n====================== KẾT QUẢ DỌN DẸP ======================")
    print(f"  * Tổng số điểm ban đầu: {len(points)}")
    print(f"  * Số điểm bị xóa trong vùng chọn (Z >= {h1}m): {deleted_count}")
    print(f"  * Số điểm còn lại: {len(pcd_cleaned.points)}")
    
    ans = input("Bạn có muốn lưu đè vào file PCD gốc không? (y/n): ")
    if ans.lower() == 'y':
        o3d.io.write_point_cloud(pcd_path, pcd_cleaned)
        print(f"Đã lưu đè thành công vào: {pcd_path}")
    else:
        dir_name = os.path.dirname(pcd_path)
        base_name = os.path.basename(pcd_path)
        name, ext = os.path.splitext(base_name)
        new_path = os.path.join(dir_name, f"{name}_clean_region{ext}")
        o3d.io.write_point_cloud(new_path, pcd_cleaned)
        print(f"Đã lưu thành file mới tại: {new_path}")
    print("=============================================================\n")


def view_pcd_with_open3d(pcd_path, use_height_color=False, edit_mode=False, filter_height=False, min_z=0.1, max_z=1.3):
    print(f"Đang đọc file PCD: {pcd_path}")
    if not os.path.exists(pcd_path):
        print(f"Lỗi: Không tìm thấy file {pcd_path}")
        return

    try:
        import open3d as o3d
    except ImportError:
        print("\n[LỖI] Chưa cài đặt thư viện 'open3d'.")
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # 1. Chế độ lọc độ cao tự động toàn bộ bản đồ
    if filter_height:
        print(f"\n=======================================================")
        print(f"Lọc độ cao toàn bản đồ: Giữ lại điểm Z từ {min_z}m đến {max_z}m")
        points = np.asarray(pcd.points)
        mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
        pcd_filtered = pcd.select_by_index(np.where(mask)[0])
        print(f"Đã lọc: Giảm từ {len(points)} điểm xuống còn {len(pcd_filtered.points)} điểm.")
        
        dir_name = os.path.dirname(pcd_path)
        base_name = os.path.basename(pcd_path)
        name, ext = os.path.splitext(base_name)
        output_path = os.path.join(dir_name, f"{name}_filtered{ext}")
        o3d.io.write_point_cloud(output_path, pcd_filtered)
        print(f"Đã lưu file PCD sạch mới tại: {output_path}")
        print("=======================================================\n")
        return

    # 2. Tô màu theo chiều cao
    if use_height_color:
        points = np.asarray(pcd.points)
        if len(points) > 0:
            z = points[:, 2]
            z_min = z.min()
            z_max = z.max()
            z_range = z_max - z_min
            if z_range > 0:
                z_norm = (z - z_min) / z_range
                colors = np.zeros((len(z), 3))
                colors[:, 0] = z_norm
                colors[:, 2] = 1.0 - z_norm
                colors[:, 1] = 1.0 - np.abs(z_norm - 0.5) * 2.0
                pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 3. Chế độ chỉnh sửa thủ công
    if edit_mode:
        print("\n=======================================================")
        print("ĐANG KHỞI ĐỘNG CHẾ ĐỘ CHỈNH SỬA BẢN ĐỒ")
        print("=======================================================\n")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Chỉnh sửa Bản đồ PCD", width=1024, height=768)
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()
        
        ans = input("Bạn có muốn lưu đè những thay đổi này không? (y/n): ")
        if ans.lower() == 'y':
            o3d.io.write_point_cloud(pcd_path, pcd)
            print(f"Đã lưu đè thành công vào: {pcd_path}")
    else:
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])
        o3d.visualization.draw_geometries(
            [pcd, coord_frame],
            window_name="Trình xem Bản đồ PCD 3D (Đỏ: X, Lục: Y, Lam: Z)",
            width=1024,
            height=768,
            left=50,
            top=50
        )

if __name__ == "__main__":
    pcd_file = DEFAULT_PCD_PATH
    
    # Chế độ chọn góc click dọn dẹp trực tiếp từ trên xuống
    if "--clean-select" in sys.argv:
        args = [a for a in sys.argv[1:] if a != "--clean-select"]
        if len(args) > 0 and not args[0].startswith("-"):
            pcd_file = args[0]
        clean_by_picked_points(pcd_file)
        sys.exit(0)
        
    # Chế độ dọn dẹp vùng nhập tay tọa độ
    if "--clean-region" in sys.argv:
        print("\n=== CHẾ ĐỘ DỌN DẸP VÙNG CHỈ ĐỊNH THEO CHIỀU CAO ===")
        try:
            min_x = float(input("Nhập Min X: "))
            max_x = float(input("Nhập Max X: "))
            min_y = float(input("Nhập Min Y: "))
            max_y = float(input("Nhập Max Y: "))
            h1    = float(input("Nhập độ cao h1 bắt đầu xóa (Z >= h1): "))
            
            args = [a for a in sys.argv[1:] if a != "--clean-region"]
            if len(args) > 0 and not args[0].startswith("-"):
                pcd_file = args[0]
                
            clean_region_by_height(pcd_file, min_x, max_x, min_y, max_y, h1)
        except ValueError:
            print("Lỗi: Nhập sai định dạng số!")
        sys.exit(0)

    # Các chế độ cũ
    use_height = False
    edit_mode = False
    filter_height = False
    min_z = 0.1
    max_z = 1.3
    
    args = sys.argv[1:]
    if "--height" in args:
        use_height = True
        args.remove("--height")
    if "--edit" in args:
        edit_mode = True
        args.remove("--edit")
    if "--filter-height" in args:
        filter_height = True
        args.remove("--filter-height")
        if "--min-z" in args:
            idx = args.index("--min-z")
            min_z = float(args[idx+1])
            args.pop(idx+1)
            args.pop(idx)
        if "--max-z" in args:
            idx = args.index("--max-z")
            max_z = float(args[idx+1])
            args.pop(idx+1)
            args.pop(idx)
        
    if len(args) > 0:
        pcd_file = args[0]
        
    view_pcd_with_open3d(pcd_file, use_height, edit_mode, filter_height, min_z, max_z)
