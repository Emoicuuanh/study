#!/usr/bin/env python3
import sys
import time

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from inspire_sdkpy import inspire_sdk
except ImportError:
    print("Error: Could not import unitree_sdk2py or inspire_sdkpy. Make sure your ROS2 workspace is sourced.")
    sys.exit(1)

# Mapping finger names
FINGERS = [
    "0: Thumb Stretch (Khớp xoay ngón cái)",
    "1: Thumb Flexion (Khớp gập ngón cái)",
    "2: Index Finger (Ngón trỏ)",
    "3: Middle Finger (Ngón giữa)",
    "4: Ring Finger (Ngón áp út)",
    "5: Pinky Finger (Ngón út)"
]

# Translation dictionaries
ERROR_MAP = {
    0: "Stall / Jammed (Đang bị kẹt/bản cực cứng kẹt)",
    1: "Over-temperature (Quá nhiệt/Nóng motor)",
    2: "Over-current (Quá dòng bảo vệ)",
    3: "Motor Abnormality (Motor bất thường)",
    4: "Communication Failure (Lỗi giao tiếp Modbus)"
}

STATUS_MAP = {
    0: "Releasing (Đang thả lỏng)",
    1: "Grasping (Đang gắp)",
    2: "Position Reached (Đã đến vị trí đích)",
    3: "Force Reached (Đã đạt lực kẹp đích)",
    5: "Current Prot. Stopped (Dừng do bảo vệ dòng)",
    6: "Stall Stopped (Dừng do kẹt cứng)",
    7: "Fault Stopped (Dừng do lỗi hệ thống)",
    255: "Error (Lỗi nghiêm trọng)"
}

def decode_errors(err_val):
    errors = []
    for bit, desc in ERROR_MAP.items():
        if err_val & (1 << bit):
            errors.append(desc)
    return errors if errors else ["None (Bình thường)"]

def get_status_str(status_val):
    return STATUS_MAP.get(status_val, f"Unknown ({status_val})")

def diagnose_hand(name, ip, lr):
    print(f"\n==========================================")
    print(f" DIAGNOSING {name} ({ip})")
    print(f"==========================================")
    
    try:
        # Initialize Modbus handler
        handler = inspire_sdk.ModbusDataHandler(
            ip=ip,
            LR=lr,
            device_id=1,
            initDDS=False
        )
        
        # Read from Modbus TCP
        data = handler.read()
        if not data or 'states' not in data:
            print(f"❌ Không thể đọc dữ liệu từ {name} ({ip}). Hãy kiểm tra dây nguồn và dây mạng.")
            return False
            
        states = data['states']
        pos_act = states.get('POS_ACT', [0]*6)
        angle_act = states.get('ANGLE_ACT', [0]*6)
        force_act = states.get('FORCE_ACT', [0]*6)
        current = states.get('CURRENT', [0]*6)
        error = states.get('ERROR', [0]*6)
        status = states.get('STATUS', [0]*6)
        temp = states.get('TEMP', [0]*6)
        
        # Print table
        header = f"{'Finger Name':<35} | {'Pos':<6} | {'Angle':<5} | {'Force':<5} | {'Curr':<5} | {'Temp':<4} | {'Status/Errors'}"
        print(header)
        print("-" * 105)
        
        has_issue = False
        for idx in range(6):
            f_name = FINGERS[idx]
            p = pos_act[idx]
            a = angle_act[idx]
            f = force_act[idx]
            c = current[idx]
            t = temp[idx]
            s = status[idx]
            e = error[idx]
            
            errs = decode_errors(e)
            status_desc = get_status_str(s)
            
            # Check if there is an active error or status warning
            issue_str = ""
            if e != 0 or s in [5, 6, 7, 255]:
                has_issue = True
                issue_str = f"⚠️ [LỖI] Err: {', '.join(errs)} | Status: {status_desc}"
            else:
                issue_str = f"OK ({status_desc})"
                
            print(f"{f_name:<35} | {p:<6} | {a:<5} | {f:<5} | {c:<5} | {t}°C  | {issue_str}")
            
        if not has_issue:
            print("\n✅ Tay hoạt động bình thường, không phát hiện lỗi motor.")
        else:
            print("\n⚠️ PHÁT HIỆN LỖI/KẸT NGÓN TAY! Hướng xử lý:")
            for idx in range(6):
                if error[idx] != 0 or status[idx] in [5, 6, 7, 255]:
                    f_name = FINGERS[idx]
                    err_list = decode_errors(error[idx])
                    print(f"  * Ngón {f_name}:")
                    if 0 in [b for b in range(5) if error[idx] & (1 << b)] or status[idx] == 6:
                        print("    -> Lỗi kẹt cơ (Stall/Stall Stopped): Kiểm tra xem khớp ngón tay có bị vướng dị vật hay vật cản cơ học không. Thử xoay/di chuyển nhẹ ngón tay bằng tay khi đã ngắt nguồn.")
                    if 1 in [b for b in range(5) if error[idx] & (1 << b)]:
                        print("    -> Lỗi quá nhiệt (Over-temperature): Tắt nguồn robot/tay khoảng 10-15 phút để motor nguội bớt rồi thử lại.")
                    if 2 in [b for b in range(5) if error[idx] & (1 << b)] or status[idx] == 5:
                        print("    -> Lỗi quá dòng (Over-current): Thường do kẹt tải nặng hoặc quá tải dòng. Hãy ngắt điện và cấp lại để reset mạch bảo vệ motor.")
                    if 4 in [b for b in range(5) if error[idx] & (1 << b)]:
                        print("    -> Lỗi giao tiếp (Communication): Hãy kiểm tra lại jack cắm hoặc cáp tín hiệu nội bộ dẫn tới ngón tay đó.")
                        
        return True
        
    except Exception as ex:
        print(f"❌ Lỗi khi chẩn đoán {name}: {ex}")
        return False

def main():
    # Init DDS Channel factory first
    ChannelFactoryInitialize(0)
    
    print("Bắt đầu chẩn đoán lỗi 2 tay robot...")
    
    # Diagnose Left Hand
    diagnose_hand("LEFT HAND (TAY TRÁI)", "192.168.123.210", "l")
    
    # Diagnose Right Hand
    diagnose_hand("RIGHT HAND (TAY PHẢI)", "192.168.123.211", "r")

if __name__ == "__main__":
    main()
