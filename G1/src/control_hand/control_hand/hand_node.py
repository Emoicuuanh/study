#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse

from unitree_msgs.action import HandCommand
from control_hand.hand_driver import InspireHandDriver
import time
# ===== Bit mask theo Inspire SDK =====
ANGLE_BIT    = 0b0001
POSITION_BIT = 0b0010
FORCE_BIT    = 0b0100
VELOCITY_BIT = 0b1000


class HandActionServer(Node):

    def __init__(self):
        super().__init__('hand_action_server')

        self.driver = InspireHandDriver()
        self._action_server = ActionServer(
            self,
            HandCommand,
            'hand_command',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("✅ Hand Action Server started")
        # self.driver.close_hand("both")

    # ==================================================
    # Goal validation
    # ==================================================
    def goal_callback(self, goal_request: HandCommand.Goal):
        self.get_logger().info(
            f"Received goal: hand={goal_request.hand}, "
            f"type={goal_request.type}, "
            f"name_action={goal_request.name_action}, "
            f"mode={goal_request.mode}"
        )

        # ---- hand ----
        if goal_request.hand not in ["left", "right", "both"]:
            self.get_logger().error("Invalid hand selection")
            return GoalResponse.REJECT

        # ---- type ----
        if goal_request.type not in ["standard", "custom"]:
            self.get_logger().error("type must be 'standard' or 'custom'")
            return GoalResponse.REJECT

        # ---- standard ----
        if goal_request.type == "standard":
            if goal_request.name_action == "":
                self.get_logger().error("name_action required for standard type")
                return GoalResponse.REJECT

        # ---- custom ----
        if goal_request.type == "custom":
            if not (0 <= goal_request.mode <= 15):
                self.get_logger().error("Invalid mode (0..15)")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    # ==================================================
    # Cancel
    # ==================================================
    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

    # ==================================================
    # Execute
    # ==================================================
    def execute_callback(self, goal_handle):
        req = goal_handle.request
        feedback = HandCommand.Feedback()

        self.get_logger().info("Executing hand action...")

        try:
            # ==================================================
            # STANDARD MODE
            # ==================================================
            if req.type == "standard":
                feedback.state = f"Executing standard action: {req.name_action}"
                goal_handle.publish_feedback(feedback)

                success = self.driver.execute_standard(
                    name=req.name_action,
                    hand=req.hand
                )

            # ==================================================
            # CUSTOM MODE
            # ==================================================
            elif req.type == "custom":
                feedback.state = f"Executing custom mode {req.mode}"
                goal_handle.publish_feedback(feedback)

                # ---- Validate by mode ----
                if req.mode & ANGLE_BIT and len(req.angles) != 6:
                    raise ValueError("Mode requires angles[6]")

                if req.mode & POSITION_BIT and len(req.positions) != 6:
                    raise ValueError("Mode requires positions[6]")

                if req.mode & FORCE_BIT and len(req.forces) != 6:
                    raise ValueError("Mode requires forces[6]")

                if req.mode & VELOCITY_BIT and len(req.speeds) != 6:
                    raise ValueError("Mode requires speeds[6]")

                success = self.driver.send_command(
                    hand=req.hand,
                    mode=req.mode,
                    angles=req.angles if req.mode & ANGLE_BIT else None,
                    positions=req.positions if req.mode & POSITION_BIT else None,
                    forces=req.forces if req.mode & FORCE_BIT else None,
                    speeds=req.speeds if req.mode & VELOCITY_BIT else None,
                    wait_time=req.timeout
                )

            else:
                raise ValueError("Invalid type")

        except Exception as e:
            self.get_logger().error(f"Execution error: {e}")
            goal_handle.abort()
            return HandCommand.Result(
                success=False,
                message=str(e)
            )

        # ==================================================
        # Result
        # ==================================================
        if success:
            feedback.state = "Completed"
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()

            return HandCommand.Result(
                success=True,
                message="Hand action executed successfully"
            )
        else:
            goal_handle.abort()
            return HandCommand.Result(
                success=False,
                message="Hand action failed"
            )


def main():
    rclpy.init()
    node = HandActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
