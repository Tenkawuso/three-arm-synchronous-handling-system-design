#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import copy
import threading

import rospy
import moveit_commander

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from tf.transformations import quaternion_from_euler

class ThreeArmSimpleDebug(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("three_arm_simple_debug", anonymous=True)

        rospy.logwarn("===== THREE ARM SIMPLE DEBUG START =====")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        self.world_frame = "world"
        self.approach_height = 0.04

        self.auto_mm_to_m = True
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        self.lock = threading.Lock()
        self.exec_lock = threading.Lock()

        self.arm_cfg = {
            "arm1": {
                "group_name": "arm1_group",
                "tcp_link": "arm1_tcp",
                "topic": "/target/P",
                "base_xyz": [0.28, 0.0, 0.0],
                "target_msg": None,
            },
            "arm2": {
                "group_name": "arm2_group",
                "tcp_link": "arm2_tcp",
                "topic": "/target/Y",
                "base_xyz": [-0.14, 0.242487, 0.0],
                "target_msg": None,
            },
            "arm3": {
                "group_name": "arm3_group",
                "tcp_link": "arm3_tcp",
                "topic": "/target/G",
                "base_xyz": [-0.14, -0.242487, 0.0],
                "target_msg": None,
            },
        }

        self.groups = {}
        for arm_id, cfg in self.arm_cfg.items():
            group = moveit_commander.MoveGroupCommander(cfg["group_name"])
            group.set_pose_reference_frame(self.world_frame)
            group.set_planning_time(8.0)
            group.set_num_planning_attempts(10)
            group.set_goal_position_tolerance(0.005)
            group.set_goal_orientation_tolerance(0.25)
            group.set_max_velocity_scaling_factor(0.1)
            group.set_max_acceleration_scaling_factor(0.1)
            group.allow_replanning(True)
            self.groups[arm_id] = group

        rospy.Subscriber("/target/P", PointStamped, self.cb_arm1, queue_size=1)
        rospy.Subscriber("/target/Y", PointStamped, self.cb_arm2, queue_size=1)
        rospy.Subscriber("/target/G", PointStamped, self.cb_arm3, queue_size=1)

        rospy.sleep(1.0)

        self.print_basic_info()
        self.setup_scene()

        rospy.loginfo("ThreeArmSimpleDebug 初始化完成")

    # =====================================================
    # Subscriber callbacks
    # =====================================================
    def cb_arm1(self, msg):
        self.update_target("arm1", msg)

    def cb_arm2(self, msg):
        self.update_target("arm2", msg)

    def cb_arm3(self, msg):
        self.update_target("arm3", msg)

    def convert_point(self, msg):
        p = copy.deepcopy(msg)

        if self.auto_mm_to_m:
            if abs(p.point.x) > 5.0 or abs(p.point.y) > 5.0 or abs(p.point.z) > 5.0:
                rospy.logwarn("检测到疑似mm单位，自动转m")
                p.point.x /= 1000.0
                p.point.y /= 1000.0
                p.point.z /= 1000.0

        p.point.x += self.offset_x
        p.point.y += self.offset_y
        p.point.z += self.offset_z
        return p

    def update_target(self, arm_id, msg):
        p = self.convert_point(msg)

        with self.lock:
            self.arm_cfg[arm_id]["target_msg"] = p

        rospy.logwarn("[%s] 收到目标点 frame=%s x=%.4f y=%.4f z=%.4f",
                      arm_id,
                      msg.header.frame_id,
                      p.point.x, p.point.y, p.point.z)

    # =====================================================
    # Basic info
    # =====================================================
    def print_basic_info(self):
        rospy.loginfo("========== BASIC INFO ==========")
        for arm_id, cfg in self.arm_cfg.items():
            rospy.loginfo("[%s] group=%s tcp=%s topic=%s base=%s",
                          arm_id,
                          cfg["group_name"],
                          cfg["tcp_link"],
                          cfg["topic"],
                          str(cfg["base_xyz"]))
            try:
                joints = self.groups[arm_id].get_active_joints()
                rospy.loginfo("[%s] active_joints=%s", arm_id, joints)
            except Exception as e:
                rospy.logerr("[%s] 获取 active joints 失败: %s", arm_id, str(e))
        rospy.loginfo("================================")

    # =====================================================
    # Scene
    # =====================================================
    def setup_scene(self):
        rospy.loginfo("添加桌面碰撞体...")

        try:
            self.scene.remove_world_object("table")
        except Exception:
            pass

        rospy.sleep(0.3)

        table = PoseStamped()
        table.header.frame_id = self.world_frame
        table.pose.orientation.w = 1.0
        table.pose.position.x = 0.0
        table.pose.position.y = 0.0
        table.pose.position.z = -0.02
        self.scene.add_box("table", table, size=(0.9, 0.9, 0.04))

        rospy.sleep(0.8)
        rospy.loginfo("桌面添加完成")

    # =====================================================
    # Target / geometry
    # =====================================================
    def wait_for_all_targets(self):
        rospy.loginfo("等待三个目标点...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            with self.lock:
                ok1 = self.arm_cfg["arm1"]["target_msg"] is not None
                ok2 = self.arm_cfg["arm2"]["target_msg"] is not None
                ok3 = self.arm_cfg["arm3"]["target_msg"] is not None

            if ok1 and ok2 and ok3:
                rospy.logwarn("三个目标点均已收到")
                return

            rospy.loginfo_throttle(
                2.0,
                "等待中: arm1=%s arm2=%s arm3=%s",
                ok1, ok2, ok3
            )
            rate.sleep()

    def get_target_xyz(self, arm_id):
        with self.lock:
            msg = self.arm_cfg[arm_id]["target_msg"]

        if msg is None:
            return None

        return [msg.point.x, msg.point.y, msg.point.z]

    def distance_to_base_xy(self, arm_id, xyz):
        bx, by, _ = self.arm_cfg[arm_id]["base_xyz"]
        x, y, _ = xyz
        return math.sqrt((x - bx) ** 2 + (y - by) ** 2)

    def target_is_reasonable(self, arm_id, xyz):
        x, y, z = xyz
        rospy.logwarn("[%s] 检查目标点合理性: %s", arm_id, str(xyz))

        if abs(x) > 0.65 or abs(y) > 0.65:
            rospy.logerr("[%s] x/y 超范围", arm_id)
            return False

        if z < -0.03 or z > 0.35:
            rospy.logerr("[%s] z 超范围", arm_id)
            return False

        d = self.distance_to_base_xy(arm_id, xyz)
        rospy.logwarn("[%s] 目标点到基座水平距离 = %.4f m", arm_id, d)

        if d > 0.38:
            rospy.logerr("[%s] 目标点距基座过远，可能不可达", arm_id)
            return False

        return True

    def compute_yaw_to_target(self, arm_id, xyz):
        bx, by, _ = self.arm_cfg[arm_id]["base_xyz"]
        x, y, _ = xyz
        return math.atan2(y - by, x - bx)

    def make_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def make_facing_pose(self, arm_id, xyz):
        yaw = self.compute_yaw_to_target(arm_id, xyz)
        rospy.logwarn("[%s] 自动生成 yaw = %.4f rad", arm_id, yaw)
        return self.make_pose(
            xyz[0], xyz[1], xyz[2],
            roll=0.0,
            pitch=0.0,
            yaw=yaw
        )

    # =====================================================
    # MoveIt
    # =====================================================
    def plan_to_pose(self, arm_id, pose):
        group = self.groups[arm_id]

        rospy.logwarn("[%s] 开始规划到目标pose...", arm_id)
        rospy.logwarn("[%s] target position = [%.4f, %.4f, %.4f]",
                      arm_id, pose.position.x, pose.position.y, pose.position.z)
        rospy.logwarn("[%s] target orientation = [%.4f, %.4f, %.4f, %.4f]",
                      arm_id,
                      pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)

        group.set_start_state_to_current_state()
        group.set_pose_target(pose, end_effector_link=self.arm_cfg[arm_id]["tcp_link"])

        for i in range(3):
            rospy.logwarn("[%s] 第 %d 次规划...", arm_id, i + 1)
            try:
                plan_result = group.plan()
            except Exception as e:
                rospy.logerr("[%s] plan() 异常: %s", arm_id, str(e))
                continue

            success = False
            traj = None

            if isinstance(plan_result, tuple):
                if len(plan_result) >= 2:
                    success = plan_result[0]
                    traj = plan_result[1]
            else:
                traj = plan_result
                try:
                    success = len(traj.joint_trajectory.points) > 0
                except Exception:
                    success = False

            if success and traj is not None:
                rospy.logwarn("[%s] 规划成功", arm_id)
                group.clear_pose_targets()
                return traj

            rospy.logwarn("[%s] 本次规划失败", arm_id)

        group.clear_pose_targets()
        rospy.logerr("[%s] 规划最终失败", arm_id)
        return None

    def execute_plan(self, arm_id, traj):
        group = self.groups[arm_id]

        rospy.logwarn("[%s] 等待执行锁...", arm_id)
        with self.exec_lock:
            rospy.logwarn("[%s] 已获得执行锁，开始执行轨迹...", arm_id)
            try:
                ok = group.execute(traj, wait=True)
                group.stop()
                group.clear_pose_targets()
                rospy.logwarn("[%s] 执行结果 = %s", arm_id, str(ok))
                return ok
            except Exception as e:
                rospy.logerr("[%s] execute() 异常: %s", arm_id, str(e))
                return False

    # =====================================================
    # Task
    # =====================================================
    def arm_task(self, arm_id):
        rospy.logwarn("[%s] ===== 任务线程启动 =====", arm_id)

        target_xyz = self.get_target_xyz(arm_id)
        rospy.logwarn("[%s] target_xyz = %s", arm_id, str(target_xyz))

        if target_xyz is None:
            rospy.logerr("[%s] 没有目标点", arm_id)
            return

        if not self.target_is_reasonable(arm_id, target_xyz):
            rospy.logerr("[%s] 目标点不合理", arm_id)
            return

        pre_grasp_xyz = [
            target_xyz[0],
            target_xyz[1],
            target_xyz[2] + self.approach_height
        ]
        rospy.logwarn("[%s] pre_grasp_xyz = %s", arm_id, str(pre_grasp_xyz))

        if not self.target_is_reasonable(arm_id, pre_grasp_xyz):
            rospy.logerr("[%s] 预抓取点不合理", arm_id)
            return

        pre_grasp_pose = self.make_facing_pose(arm_id, pre_grasp_xyz)

        traj = self.plan_to_pose(arm_id, pre_grasp_pose)
        if traj is None:
            rospy.logerr("[%s] 预抓取规划失败", arm_id)
            return

        ok = self.execute_plan(arm_id, traj)
        if not ok:
            rospy.logerr("[%s] 预抓取执行失败", arm_id)
            return

        rospy.logwarn("[%s] ===== 到达预抓取点成功 =====", arm_id)

    # =====================================================
    # Main
    # =====================================================
    def run(self):
        self.wait_for_all_targets()

        threads = []
        for arm_id in ["arm1", "arm2", "arm3"]:
            t = threading.Thread(target=self.arm_task, args=(arm_id,))
            t.daemon = True
            t.start()
            threads.append(t)

        for t in threads:
            t.join()

        rospy.logwarn("===== 三臂简化联调结束 =====")

if __name__ == "__main__":
    try:
        node = ThreeArmSimpleDebug()
        rospy.sleep(1.0)
        node.run()
    except rospy.ROSInterruptException:
        pass