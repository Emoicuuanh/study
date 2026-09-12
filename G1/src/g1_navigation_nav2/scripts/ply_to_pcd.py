#!/usr/bin/env python3
import os
import sys
import argparse

# Đường dẫn mặc định
DEFAULT_INPUT_PLY = "/home/hoangdc/ROS2/unitree_G1/maps/g1_office/g1_office_aligned_filter.ply"

def main():
    parser = argparse.ArgumentParser(description="Chuyển đổi file Point Cloud từ định dạng .ply sang .pcd.")
    parser.add_argument("-i", "--input", default=DEFAULT_INPUT_PLY, help="Đường dẫn file PLY đầu vào")
    parser.add_argument("-o", "--output", default=None, help="Đường dẫn file PCD đầu ra (mặc định tự động tạo từ file đầu vào)")
    args = parser.parse_args()

    input_path = args.input
    
    # Kiểm tra sự tồn tại của file đầu vào
    if not os.path.exists(input_path):
        print(f"[LỖI] Không tìm thấy file PLY tại: {input_path}")
        sys.exit(1)

    # Tự động tạo đường dẫn file đầu ra nếu không chỉ định
    if args.output is None:
        dir_name = os.path.dirname(input_path)
        base_name = os.path.basename(input_path)
        name, _ = os.path.splitext(base_name)
        output_path = os.path.join(dir_name, f"{name}.pcd")
    else:
        output_path = args.output

    print(f"Đang đọc file PLY: {input_path}")
    
    try:
        import open3d as o3d
    except ImportError:
        print("\n[LỖI] Chưa cài đặt thư viện 'open3d'. Hãy chạy: pip install open3d")
        sys.exit(1)

    try:
        pcd = o3d.io.read_point_cloud(input_path)
        
        # Kiểm tra mây điểm có rỗng không
        if pcd.is_empty():
            print(f"[CẢNH BÁO] Bản đồ hoặc mây điểm đọc được từ {input_path} bị rỗng!")
            
        num_points = len(pcd.points)
        print(f"-> Đã đọc thành công mây điểm có {num_points} điểm.")
        
        print(f"Đang ghi sang định dạng PCD tại: {output_path} ...")
        success = o3d.io.write_point_cloud(output_path, pcd)
        
        if success:
            print(f"[THÀNH CÔNG] Đã chuyển đổi và lưu file PCD thành công!")
        else:
            print(f"[LỖI] Ghi file PCD thất bại.")
            sys.exit(1)

    except Exception as e:
        print(f"[LỖI] Có lỗi xảy ra trong quá trình chuyển đổi: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
