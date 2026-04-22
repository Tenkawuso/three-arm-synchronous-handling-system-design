#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import time
from geometry_msgs.msg import PointStamped

class SingleArmTest:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('single_arm_test', anonymous=True)

        self.arm = moveit_commander.MoveGroupCommander("arm1_group")
        self.arm.set_pose_reference_frame("world")
        self.arm.allow_replanning(True)

        # 加速设置
        self.arm.set_planning_time(5.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.set_planner_id("RRTstar")

        # 夹爪参数
        self.gripper_joint = "arm1_J611"
        self.open_angle = -0.5236          # 张开 30°
        self.close_angle = 0.0             # 闭合

        # 侧移偏移量
        self.side_offset_x = 0.05
        self.side_offset_y = -0.03
        self.side_height = 0.04

        self.target = None
        rospy.Subscriber('/target/P', PointStamped, self.cb)
        rospy.loginfo("等待目标点...")
        while not rospy.is_shutdown() and self.target is None:
            rospy.sleep(0.1)
        rospy.loginfo("收到目标点，开始执行...")
        self.run()

    def cb(self, msg):
        self.target = msg

    def set_gripper(self, angle):
        """设置夹爪关节角度，保持其他关节不变"""
        joint_names = self.arm.get_active_joints()
        current_values = self.arm.get_current_joint_values()
        target = {}
        for name, val in zip(joint_names, current_values):
            target[name] = val
        target[self.gripper_joint] = angle
        self.arm.set_joint_value_target(target)
        success = self.arm.go(wait=True)
        if success:
            rospy.loginfo(f"夹爪关节 {self.gripper_joint} 运动到 {angle:.3f} 弧度")
        else:
            rospy.logerr(f"夹爪关节运动失败")
        self.arm.stop()
        self.arm.clear_pose_targets()

    def go_home(self):
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.set_joint_value_target(home_joints)
        success = self.arm.go(wait=True)
        if success:
            rospy.loginfo("已回家")
        else:
            rospy.logwarn("无法回到 home，继续")
        self.arm.stop()
        self.arm.clear_pose_targets()

    def move_to_pose(self, x, y, z, desc="规划"):
        """移动到指定坐标，使用固定姿态"""
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = 0.01947845
        target_pose.orientation.y = 0.00088976
        target_pose.orientation.z = 0.99742708
        target_pose.orientation.w = 0.06898569

        rospy.loginfo(f"{desc}: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.arm.set_pose_target(target_pose)
        success = self.arm.go(wait=True)
        if success:
            rospy.loginfo(f"{desc}成功")
        else:
            rospy.logerr(f"{desc}失败")
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def move_to_side(self):
        """移动到方块侧上方（XY偏移，Z固定为0.04）"""
        block_x = self.target.point.x
        block_y = self.target.point.y

        target_x = block_x + self.side_offset_x
        target_y = block_y + self.side_offset_y
        target_z = self.side_height

        rospy.loginfo("目标位置（侧上方）: (%.3f, %.3f, %.3f)", target_x, target_y, target_z)
        success = self.move_to_pose(target_x, target_y, target_z, "移动到侧上方")
        if success:
            self.set_gripper(self.open_angle)
            time.sleep(0.5)
        return success

    def lower_to_grasp(self):
        """垂直下降到抓取高度，保持当前末端姿态"""
        current_pose = self.arm.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position = current_pose.position
        target_pose.position.z = 0.02   # 抓取高度
        target_pose.orientation = current_pose.orientation
        rospy.loginfo(f"垂直下移到抓取高度: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        self.arm.set_pose_target(target_pose)
        success = self.arm.go(wait=True)
        if success:
            rospy.loginfo("垂直下移成功")
        else:
            rospy.logerr("垂直下移失败")
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def lift_up(self, delta_z=0.05):
        """垂直抬升末端，保持当前姿态"""
        current_pose = self.arm.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position = current_pose.position
        target_pose.position.z += delta_z
        target_pose.orientation = current_pose.orientation
        rospy.loginfo(f"抬升: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        self.arm.set_pose_target(target_pose)
        success = self.arm.go(wait=True)
        if success:
            rospy.loginfo("抬升成功")
        else:
            rospy.logerr("抬升失败")
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def lower_to_place(self):
        """垂直下降到放置高度（Z=0.02），保持当前姿态"""
        current_pose = self.arm.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position = current_pose.position
        target_pose.position.z = 0.02
        target_pose.orientation = current_pose.orientation
        rospy.loginfo(f"垂直下移到放置高度: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        self.arm.set_pose_target(target_pose)
        success = self.arm.go(wait=True)
        if success:
            rospy.loginfo("垂直下移成功")
        else:
            rospy.logerr("垂直下移失败")
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def run(self):
        # 1. 回家
        self.go_home()

        # 2. 移动到侧上方并张开夹爪
        if not self.move_to_side():
            rospy.logerr("移动到侧上方失败，退出")
            return

        time.sleep(2)

        # 3. 垂直下移到抓取高度
        if not self.lower_to_grasp():
            rospy.logerr("下移失败，退出")
            return

        # 4. 闭合夹爪抓取
        rospy.loginfo("闭合夹爪...")
        self.set_gripper(self.close_angle)
        time.sleep(0.5)

        # 5. 抬升
        if not self.lift_up(delta_z=0.05):
            rospy.logerr("抬升失败，退出")
            return

        # 6. 回家
        self.go_home()

        # 7. 保持姿态，垂直下降到 0.02 米处
        if not self.lower_to_place():
            rospy.logerr("垂直下降失败，退出")
            return

        # 8. 松开夹爪放置
        rospy.loginfo("松开夹爪...")
        self.set_gripper(self.open_angle)
        time.sleep(0.5)

        rospy.loginfo("抓取放置完成")

if __name__ == '__main__':
    try:
        SingleArmTest()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))