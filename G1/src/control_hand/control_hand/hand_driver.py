#!/usr/bin/env python3
import sys
import time
import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds, inspire_sdk

# ===== Bit mask theo Inspire SDK =====
ANGLE_BIT    = 0b0001
POSITION_BIT = 0b0010
FORCE_BIT    = 0b0100
VELOCITY_BIT = 0b1000


class InspireHandDriver:
    """
    Low-level driver for Inspire Hand.
    - DDS publisher + Modbus client
    - Chỉ chịu trách nhiệm gửi lệnh & preset
    """

    def __init__(self):
        # ===== DDS init =====

        ChannelFactoryInitialize(0)

        self.pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
        self.pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)

        self.pub_l.Init()
        self.pub_r.Init()
        print("[HandDriver] Waiting 2s for DDS handshake...")
        time.sleep(2.0)

        # ===== Map STANDARD actions =====
        self.standard_actions = {
            "open_hand": self.open_hand,
            "close_hand": self.close_hand,
            "take_board": self.take_board,
            "place_board": self.place_board,
        }

    # =====================================================
    # Utility
    # =====================================================
    def _validate_array(self, arr, name: str):
        if arr is None:
            raise ValueError(f"{name} required by mode but not provided")
        if len(arr) != 6:
            raise ValueError(f"{name} must have length 6")

    # =====================================================
    # CUSTOM COMMAND (mode-based)
    # =====================================================
    def send_command(self, hand: str, mode: int, angles=None, positions=None, forces=None, speeds=None, wait_time: float = 1.0) -> bool:
        """
        Gửi lệnh custom theo mode Inspire SDK
        """
        print(f"[DEBUG] Sending to {hand}: mode={mode}, angles={angles}, forces={forces}, speeds={speeds}")

        if not (0 <= mode <= 15):
            raise ValueError("Mode must be in range 0..15")

        cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        cmd.mode = mode

        # ===== ANGLE =====
        if mode & ANGLE_BIT:
            self._validate_array(angles, "angles")
            cmd.angle_set = np.clip(angles, 0, 1000).tolist()

        # ===== POSITION =====
        if mode & POSITION_BIT:
            self._validate_array(positions, "positions")
            cmd.pos_set = positions  # type: ignore

        # ===== FORCE =====
        if mode & FORCE_BIT:
            self._validate_array(forces, "forces")
            cmd.force_set = forces  # type: ignore

        # ===== VELOCITY =====
        if mode & VELOCITY_BIT:
            self._validate_array(speeds, "speeds")
            cmd.speed_set = speeds  # type: ignore

        try:
            if hand in ["left", "both"]:
                self.pub_l.Write(cmd)
            if hand in ["right", "both"]:
                self.pub_r.Write(cmd)

            time.sleep(wait_time)
            return True

        except Exception as e:
            print("[HandDriver ERROR]", e)
            return False

    # =====================================================
    # STANDARD ACTION INTERFACE
    # =====================================================
    def execute_standard(self, name: str, hand: str) -> bool:
        if name not in self.standard_actions:
            raise ValueError(f"Unknown standard action: {name}")
        return self.standard_actions[name](hand)

    # =====================================================
    # PRESET ACTIONS
    # =====================================================
    def open_hand(self, hand: str = "both") -> bool:
        print("Open hand command issued")
        return self.send_command(
            hand=hand,
            mode=0b1101,
            angles=[1000, 1000, 1000, 1000, 1000, 0],
            forces=[1000] * 6,
            speeds=[400] * 6,
            wait_time=3.0
        )

    def close_hand(self, hand: str = "both") -> bool:
        print("Close hand command issued")
        return self.send_command(
            hand=hand,
            mode=0b1101,
            angles=[0, 0, 0, 0, 0, 1000],
            forces=[500] * 6,
            speeds=[400, 400, 400, 400, 400, 400],
            wait_time=3.0
        )
    def take_board(self, hand: str = "both") -> bool:
        print("Take board command issued")
        return self.send_command(
            hand=hand,
            mode=0b1101,
            angles=[0, 0, 0, 0, 0, 0],
            forces=[1000] * 6,
            speeds=[400, 400, 400, 400, 180, 400],
            wait_time=3.0
        )
    def place_board(self, hand: str = "both") -> bool:
        print("Place board command issued")
        return self.send_command(
            hand=hand,
            mode=0b1101,
            angles=[200, 200, 200, 200, 200, 1000],
            forces=[500] * 6,
            speeds=[300] * 6,
            wait_time=3.0
        )









# #!/usr/bin/env python3
# import sys
# import time
# import numpy as np

# from unitree_sdk2py.core.channel import (
#     ChannelPublisher,
#     ChannelFactoryInitialize
# )
# from inspire_sdkpy import inspire_hand_defaut, inspire_dds

# # ===== Bit mask theo Inspire SDK =====
# ANGLE_BIT    = 0b0001
# POSITION_BIT = 0b0010
# FORCE_BIT    = 0b0100
# VELOCITY_BIT = 0b1000


# class InspireHandDriver:
#     """
#     Low-level driver for Inspire Hand.
#     - Không phụ thuộc ROS
#     - Chỉ chịu trách nhiệm gửi lệnh & preset
#     """

#     def __init__(self):
#         # BẮT BUỘC
#         # ChannelFactoryInitialize(0)
#         if len(sys.argv)>1:
#             print("Use config file:", sys.argv[1])
#             ChannelFactoryInitialize(0, sys.argv[1])
#         else:
#             print("Use default config file")
#             ChannelFactoryInitialize(0)
#         self.pub_l = ChannelPublisher(
#             "rt/inspire_hand/ctrl/l",
#             inspire_dds.inspire_hand_ctrl,

#         )
#         self.pub_r = ChannelPublisher(
#             "rt/inspire_hand/ctrl/r",
#             inspire_dds.inspire_hand_ctrl,

#         )

#         # self.pub_l = ChannelPublisher(
#         #     "rt/inspire_hand/ctrl/l",
#         #     inspire_dds.inspire_hand_ctrl
#         # )
#         # self.pub_r = ChannelPublisher(
#         #     "rt/inspire_hand/ctrl/r",
#         #     inspire_dds.inspire_hand_ctrl
#         # )

#         self.pub_l.Init()
#         self.pub_r.Init()
#         time.sleep(2.0)  # đợi DDS handshake
#         # ===== Map STANDARD actions =====
#         self.standard_actions = {
#             "open_hand": self.open_hand,
#             "close_hand": self.close_hand,
#         }

#     # =====================================================
#     # Utility
#     # =====================================================
#     def _validate_array(self, arr, name: str):
#         if arr is None:
#             raise ValueError(f"{name} required by mode but not provided")
#         if len(arr) != 6:
#             raise ValueError(f"{name} must have length 6")

#     # =====================================================
#     # CUSTOM COMMAND (mode-based)
#     # =====================================================
#     def send_command(
#         self,
#         hand: str,
#         mode: int,
#         angles=None,
#         positions=None,
#         forces=None,
#         speeds=None,
#         wait_time: float = 1.0
#     ) -> bool:
#         """
#         Gửi lệnh custom theo mode Inspire SDK
#         """
#         print(f"[DEBUG] Sending to {hand}: mode={mode}, angles={angles}, forces={forces}, speeds={speeds}")
        

#         if not (0 <= mode <= 15):
#             raise ValueError("Mode must be in range 0..15")

#         cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
#         cmd.mode = mode

#         # ===== ANGLE =====
#         if mode & ANGLE_BIT:
#             self._validate_array(angles, "angles")
#             cmd.angle_set = np.clip(angles, 0, 1000).tolist()

#         # ===== POSITION =====
#         if mode & POSITION_BIT:
#             self._validate_array(positions, "positions")
#             cmd.pos_set = positions  # type: ignore

#         # ===== FORCE =====
#         if mode & FORCE_BIT:
#             self._validate_array(forces, "forces")
#             cmd.force_set = forces  # type: ignore

#         # ===== VELOCITY =====
#         if mode & VELOCITY_BIT:
#             self._validate_array(speeds, "speeds")
#             cmd.speed_set = speeds  # type: ignore

#         try:
#             if hand in ["left", "both"]:
#                 print("Sending command to left hand:", cmd)
#                 self.pub_l.Write(cmd)

#             if hand in ["right", "both"]:
#                 print("Sending command to right hand:", cmd)
#                 self.pub_r.Write(cmd)

#             time.sleep(wait_time)
#             return True

#         except Exception as e:
#             print("[HandDriver ERROR]", e)
#             return False

#     # =====================================================
#     # STANDARD ACTION INTERFACE
#     # =====================================================
#     def execute_standard(self, name: str, hand: str) -> bool:
#         """
#         Gọi preset action theo tên
#         """
#         if name not in self.standard_actions:
#             raise ValueError(f"Unknown standard action: {name}")

#         return self.standard_actions[name](hand)

#     # =====================================================
#     # PRESET ACTIONS (AN TOÀN)
#     # =====================================================
#     def close_hand(self, hand: str = "both") -> bool:
#         """
#         Close hand – grasp chuẩn
#         Mode 13: angle + force + velocity
#         """
#         print("Close hand command issued")
#         return self.send_command(
#             hand=hand,
#             mode=0b1101,
#             angles=[1000, 1000, 1000, 1000, 1000, 0],
#             forces=[1000] * 6,
#             speeds=[400] * 6,
#             wait_time=2.0
#         )

#     def open_hand(self, hand: str = "both") -> bool:
#         """
#         Open hand – release
#         Mode 13: angle + force + velocity
#         """
#         print("Open hand command issued")
#         return self.send_command(
#             hand=hand,
#             mode=0b1101,
#             angles=[0, 0, 0, 0, 0, 1000],
#             forces=[500] * 6,
#             speeds=[400, 400, 400, 400, 180, 400],
#             wait_time=2.0
#         )
