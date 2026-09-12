#!/usr/bin/env python3
import time
import traceback

from inspire_sdkpy import inspire_sdk
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


# =========================
# DDS INIT – ONLY ONCE
# =========================
ChannelFactoryInitialize(0)


class HandDriver:
    def __init__(self, name, ip, LR):
        self.name = name
        self.ip = ip
        self.LR = LR

        self.handler = None
        self.connected = False

        # watchdog
        self.last_ok_time = 0
        self.reconnect_interval = 5

        # hz monitor
        self.read_count = 0
        self.last_hz_time = time.time()

    # =========================
    # CONNECT
    # =========================
    def connect(self):
        try:
            print(f"[{self.name}] Connecting to {self.ip} ...")

            self.handler = inspire_sdk.ModbusDataHandler(
                ip=self.ip,
                LR=self.LR,
                device_id=1,
                initDDS=False,     # 🚨 DDS đã init ở main
                max_retries=1,
                retry_delay=0
            )

            if not hasattr(self.handler, "pub"):
                self.handler.pub = None

            self.connected = True
            self.last_ok_time = time.time()
            self.read_count = 0
            self.last_hz_time = time.time()

            print(f"[{self.name}] Connected OK")

        except Exception as e:
            self.connected = False
            self.handler = None
            print(f"[{self.name}] Connect failed: {e}")

    # =========================
    # READ LOOP
    # =========================
    def read(self):
        if not self.connected or self.handler is None:
            return

        try:
            self.handler.read()

            self.last_ok_time = time.time()
            self.read_count += 1

            # Hz print every 1s
            now = time.time()
            if now - self.last_hz_time >= 1.0:
                hz = self.read_count / (now - self.last_hz_time)
                print(f"[{self.name}] Hz: {hz:.1f}")
                self.read_count = 0
                self.last_hz_time = now

        except Exception as e:
            print(f"[{self.name}] Read error → lost connection")
            print(e)
            self.disconnect()

    # =========================
    # DISCONNECT
    # =========================
    def disconnect(self):
        self.connected = False
        self.handler = None

    # =========================
    # WATCHDOG
    # =========================
    def watchdog(self):
        if self.connected:
            return

        if time.time() - self.last_ok_time < self.reconnect_interval:
            return

        self.last_ok_time = time.time()
        self.connect()


# =========================
# MAIN
# =========================
def main():
    right_hand = HandDriver(
        name="RIGHT_HAND",
        ip="192.168.123.211",
        LR="r"
    )

    left_hand = HandDriver(
        name="LEFT_HAND",
        ip="192.168.123.210",
        LR="l"
    )

    print("[SYSTEM] Hand driver started")

    while True:
        try:
            # watchdog
            right_hand.watchdog()
            left_hand.watchdog()

            # read loop
            right_hand.read()
            left_hand.read()

            time.sleep(0.01)

        except KeyboardInterrupt:
            print("[SYSTEM] Shutdown requested")
            break

        except Exception:
            print("[SYSTEM] Unexpected error")
            traceback.print_exc()
            time.sleep(1)


if __name__ == "__main__":
    main()





# import multiprocessing
# import time
# from inspire_sdkpy import inspire_sdk, inspire_hand_defaut

# def worker(ip,LR,name,network=None):
#     handler=inspire_sdk.ModbusDataHandler(network=network,ip=ip, LR=LR, device_id=1)

#     call_count = 0
#     start_time = time.perf_counter()
#     time.sleep(0.5)
    
#     try:
#         while True:
#             data_dict = handler.read()
#             call_count += 1
#             time.sleep(0.001)
            
#             if call_count % 10 == 0:
#                 elapsed_time = time.perf_counter() - start_time
#                 frequency = call_count / elapsed_time
#                 print(f"{name} 当前频率: {frequency:.2f} Hz, 调用次数: {call_count}, 耗时: {elapsed_time:.6f} 秒")
#     except KeyboardInterrupt:
#         elapsed_time = time.perf_counter() - start_time
#         frequency = call_count / elapsed_time if elapsed_time > 0 else 0
#         print(f"{name} 程序结束. 总调用次数: {call_count}, 总耗时: {elapsed_time:.6f} 秒, 最终频率: {frequency:.2f} Hz")

# if __name__ == "__main__":
#     # 使用默认IP地址的示例

#     process_r = multiprocessing.Process(target=worker, args=('192.168.123.211','r',"右手进程"))
#     process_l = multiprocessing.Process(target=worker, args=('192.168.123.210','l',"左手进程"))

#     process_r.start()
#     time.sleep(0.6)
#     process_l.start()

#     try:
#         while True:
#             time.sleep(10)
#     except KeyboardInterrupt:
#         process_r.terminate()
#         process_l.terminate()






# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import time
# from inspire_sdkpy import inspire_sdk

# class HandDriver(Node):
#     def __init__(self, ip, LR, name, network=None):
#         super().__init__(name)

#         self.handler = inspire_sdk.ModbusDataHandler(
#             network=network,
#             ip=ip,
#             LR=LR,
#             device_id=1
#         )
#         # ⚠️ cực kỳ quan trọng
#         self.handler.init_ros(self)
#         self.handler.init_modbus()
#         self.handler.start()
#         # 🚑 chặn publish nội bộ của SDK
#         self.handler.pub = None

#         self.call_count = 0
#         self.start_time = time.perf_counter()

#         # ✅ 20–50 Hz là hợp lý
#         self.timer = self.create_timer(0.05, self.read_loop)

#     def read_loop(self):
#         try:
#             data = self.handler.read()
#             self.call_count += 1

#             if self.call_count % 10 == 0:
#                 elapsed = time.perf_counter() - self.start_time
#                 freq = self.call_count / elapsed
#                 self.get_logger().info(
#                     f"{self.get_name()} freq={freq:.2f}Hz"
#                 )

#         except Exception as e:
#             self.get_logger().error(str(e))


# def main(args=None):
#     rclpy.init(args=args)

#     # Initialize two hand drivers
#     right_hand = HandDriver('192.168.123.211', 'r', 'right_hand')
#     left_hand = HandDriver('192.168.123.210', 'l', 'left_hand')


#     # MultiThreadedExecutor to run both hands concurrently
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(right_hand)
#     executor.add_node(left_hand)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         right_hand.get_logger().info("Right Hand stopped")
#         left_hand.get_logger().info("Left Hand stopped")
#     finally:
#         right_hand.destroy_node()
#         left_hand.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
