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

class ThreeArmHoistV1(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("three_arm_hoist_v1", anonymous=True)

        rospy.logwarn("===== THREE ARM HOIST V1 START =====")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        self.world_frame = "world"

        # -------------------------------------------------
        # 参数
        # -------------------------------------------------
        self.safe_height = 0.03           # 安全层高度
        self.final_target_offset_z = 0.0  # 最终抓取点偏移

        self.auto_mm_to_m = True
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        # 夹爪参数
        self.gripper_open_angle = -math.radians(60.0)
        self.gripper_close_angle = 0.0

        # 避碰参数
        self.conflict_xy_threshold = 0.08
        self.wait_sleep = 0.2
        self.max_wait_time = 30.0

        # 优先级：数值越小优先级越高
        self.priority_order = {
            "arm1": 1,
            "arm2": 2,
            "arm3": 3,
        }

        self.lock = threading.Lock()
        self.plan_lock = threading.Lock()   # 新增：规划锁
        self.exec_lock = threading.Lock()
        self.state_lock = threading.Lock()

        self.arm_cfg = {
            "arm1": {
                "arm_group_name": "arm1_group",
                "gripper_group_name": "arm1_gripper",
                "tcp_link": "arm1_tcp",
                "topic": "/target/P",
                "base_xyz": [0.30, 0.0, 0.0],
                "gripper_joint": "arm1_J611",
                "target_msg": None,
                "safe_joints": None,
            },
            "arm2": {
                "arm_group_name": "arm2_group",
                "gripper_group_name": "arm2_gripper",
                "tcp_link": "arm2_tcp",
                "topic": "/target/Y",
                "base_xyz": [-0.15, 0.2598076, 0.0],
                "gripper_joint": "arm2_J611",
                "target_msg": None,
                "safe_joints": None,
            },
            "arm3": {
                "arm_group_name": "arm3_group",
                "gripper_group_name": "arm3_gripper",
                "tcp_link": "arm3_tcp",
                "topic": "/target/G",
                "base_xyz": [-0.15, -0.2598076, 0.0],
                "gripper_joint": "arm3_J611",
                "target_msg": None,
                "safe_joints": None,
            },
        }

        self.arm_groups = {}
        self.gripper_groups = {}
        self.target_subs = {}

        self.arm_states = {
            "arm1": "idle",
            "arm2": "idle",
            "arm3": "idle",
        }
        # idle / moving_safe / safe_ready / waiting_for_grasp /
        # entering_grasp / grasped / failed

        for arm_id, cfg in self.arm_cfg.items():
            arm_group = moveit_commander.MoveGroupCommander(cfg["arm_group_name"])
            arm_group.set_pose_reference_frame(self.world_frame)
            arm_group.set_end_effector_link(cfg["tcp_link"])
            arm_group.set_planning_time(12.0)
            arm_group.set_num_planning_attempts(20)
            arm_group.set_goal_position_tolerance(0.005)
            arm_group.set_goal_orientation_tolerance(0.8)
            arm_group.set_max_velocity_scaling_factor(0.1)
            arm_group.set_max_acceleration_scaling_factor(0.1)
            arm_group.allow_replanning(True)
            self.arm_groups[arm_id] = arm_group

            gripper_group = moveit_commander.MoveGroupCommander(cfg["gripper_group_name"])
            gripper_group.set_planning_time(5.0)
            gripper_group.set_num_planning_attempts(10)
            gripper_group.set_max_velocity_scaling_factor(0.5)
            gripper_group.set_max_acceleration_scaling_factor(0.5)
            gripper_group.allow_replanning(True)
            self.gripper_groups[arm_id] = gripper_group

        # 保存 subscriber 句柄，后面可取消订阅
        self.target_subs["arm1"] = rospy.Subscriber("/target/P", PointStamped, self.cb_arm1, queue_size=1)
        self.target_subs["arm2"] = rospy.Subscriber("/target/Y", PointStamped, self.cb_arm2, queue_size=1)
        self.target_subs["arm3"] = rospy.Subscriber("/target/G", PointStamped, self.cb_arm3, queue_size=1)

        rospy.sleep(1.0)
        self.print_basic_info()
        self.setup_scene()
        self.capture_initial_safe_joints()

        rospy.loginfo("ThreeArmHoistV1 初始化完成")

    # =====================================================
    # Subscriber
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
                rospy.logwarn("检测到疑似 mm 单位，自动转 m")
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
                      arm_id, msg.header.frame_id,
                      p.point.x, p.point.y, p.point.z)

    def unregister_target_subscribers(self):
        rospy.logwarn("已收齐三个目标点，取消订阅 /target/* ，冻结本轮任务目标")
        for arm_id, sub in self.target_subs.items():
            try:
                if sub is not None:
                    sub.unregister()
                    rospy.logwarn("[%s] 已取消订阅 %s", arm_id, self.arm_cfg[arm_id]["topic"])
            except Exception as e:
                rospy.logerr("[%s] 取消订阅失败: %s", arm_id, str(e))
        self.target_subs = {}

    # =====================================================
    # Info / Scene
    # =====================================================
    def print_basic_info(self):
        rospy.loginfo("========== BASIC INFO ==========")
        for arm_id, cfg in self.arm_cfg.items():
            rospy.loginfo("[%s] arm_group=%s gripper_group=%s tcp=%s topic=%s base=%s joint=%s priority=%d",
                          arm_id,
                          cfg["arm_group_name"],
                          cfg["gripper_group_name"],
                          cfg["tcp_link"],
                          cfg["topic"],
                          str(cfg["base_xyz"]),
                          cfg["gripper_joint"],
                          self.priority_order[arm_id])
            try:
                rospy.loginfo("[%s] arm active_joints=%s",
                              arm_id, self.arm_groups[arm_id].get_active_joints())
            except Exception as e:
                rospy.logerr("[%s] arm active_joints 获取失败: %s", arm_id, str(e))
            try:
                rospy.loginfo("[%s] gripper active_joints=%s",
                              arm_id, self.gripper_groups[arm_id].get_active_joints())
            except Exception as e:
                rospy.logerr("[%s] gripper active_joints 获取失败: %s", arm_id, str(e))
        rospy.loginfo("================================")

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

    def capture_initial_safe_joints(self):
        rospy.logwarn("开始记录三条机械臂启动时的当前姿态，作为安全位...")
        rospy.sleep(1.0)

        for arm_id in ["arm1", "arm2", "arm3"]:
            try:
                vals = self.arm_groups[arm_id].get_current_joint_values()
                self.arm_cfg[arm_id]["safe_joints"] = vals
                rospy.logwarn("[%s] 初始安全关节位 = %s", arm_id, str(vals))
            except Exception as e:
                rospy.logerr("[%s] 记录初始安全关节位失败: %s", arm_id, str(e))
                self.arm_cfg[arm_id]["safe_joints"] = None

    # =====================================================
    # Target / Geometry
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
                self.unregister_target_subscribers()
                return

            rospy.loginfo_throttle(2.0, "等待中: arm1=%s arm2=%s arm3=%s", ok1, ok2, ok3)
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
        return self.make_pose(xyz[0], xyz[1], xyz[2], 0.0, 0.0, yaw)

    def make_safe_pose(self, arm_id, target_xyz):
        safe_xyz = [target_xyz[0], target_xyz[1], target_xyz[2] + self.safe_height]
        return self.make_facing_pose(arm_id, safe_xyz)

    def xy_distance(self, xyz1, xyz2):
        return math.sqrt((xyz1[0] - xyz2[0]) ** 2 + (xyz1[1] - xyz2[1]) ** 2)

    # =====================================================
    # State
    # =====================================================
    def set_arm_state(self, arm_id, state):
        with self.state_lock:
            self.arm_states[arm_id] = state
        rospy.logwarn("[%s] 状态 -> %s", arm_id, state)

    def get_arm_state(self, arm_id):
        with self.state_lock:
            return self.arm_states[arm_id]

    # =====================================================
    # Planning / Execute - Arm Pose
    # =====================================================
    def plan_arm_to_pose(self, arm_id, pose):
        group = self.arm_groups[arm_id]

        rospy.logwarn("[%s] 开始规划机械臂到目标 pose...", arm_id)
        rospy.logwarn("[%s] target position = [%.4f, %.4f, %.4f]",
                      arm_id, pose.position.x, pose.position.y, pose.position.z)
        rospy.logwarn("[%s] target orientation = [%.4f, %.4f, %.4f, %.4f]",
                      arm_id,
                      pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)

        with self.plan_lock:
            group.set_start_state_to_current_state()
            group.set_pose_target(pose, end_effector_link=self.arm_cfg[arm_id]["tcp_link"])

            for i in range(3):
                rospy.logwarn("[%s] 第 %d 次机械臂规划...", arm_id, i + 1)
                try:
                    plan_result = group.plan()
                except Exception as e:
                    rospy.logerr("[%s] arm plan() 异常: %s", arm_id, str(e))
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
                    rospy.logwarn("[%s] 机械臂规划成功", arm_id)
                    group.clear_pose_targets()
                    return traj

                rospy.logwarn("[%s] 本次机械臂规划失败", arm_id)

            group.clear_pose_targets()

        rospy.logerr("[%s] 机械臂规划最终失败", arm_id)
        return None

    # =====================================================
    # Planning / Execute - Arm Joint
    # =====================================================
    def plan_arm_to_joint_values(self, arm_id, joint_values):
        group = self.arm_groups[arm_id]

        rospy.logwarn("[%s] 开始规划机械臂到关节位: %s", arm_id, str(joint_values))

        try:
            active_joints = group.get_active_joints()
            rospy.logwarn("[%s] arm active_joints = %s", arm_id, active_joints)
        except Exception as e:
            rospy.logerr("[%s] 获取 arm active_joints 失败: %s", arm_id, str(e))
            return None

        if len(active_joints) != len(joint_values):
            rospy.logerr("[%s] safe_joints 数量(%d) 与 active_joints 数量(%d) 不一致",
                         arm_id, len(joint_values), len(active_joints))
            return None

        with self.plan_lock:
            group.set_start_state_to_current_state()

            try:
                group.set_joint_value_target(joint_values)
            except Exception as e:
                rospy.logerr("[%s] set_joint_value_target(joint_values) 失败: %s", arm_id, str(e))
                return None

            for i in range(3):
                rospy.logwarn("[%s] 第 %d 次关节位规划...", arm_id, i + 1)
                try:
                    plan_result = group.plan()
                except Exception as e:
                    rospy.logerr("[%s] joint plan() 异常: %s", arm_id, str(e))
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
                    rospy.logwarn("[%s] 关节位规划成功", arm_id)
                    return traj

                rospy.logwarn("[%s] 本次关节位规划失败", arm_id)

        rospy.logerr("[%s] 关节位规划最终失败", arm_id)
        return None

    def execute_arm_plan(self, arm_id, traj):
        group = self.arm_groups[arm_id]

        rospy.logwarn("[%s] 等待执行锁(机械臂)...", arm_id)
        with self.exec_lock:
            rospy.logwarn("[%s] 已获得执行锁，开始执行机械臂轨迹...", arm_id)
            try:
                ok = group.execute(traj, wait=True)
                group.stop()
                group.clear_pose_targets()
                rospy.logwarn("[%s] 机械臂执行结果 = %s", arm_id, str(ok))
                return ok
            except Exception as e:
                rospy.logerr("[%s] arm execute() 异常: %s", arm_id, str(e))
                return False

    def move_arm_to_pose(self, arm_id, pose):
        traj = self.plan_arm_to_pose(arm_id, pose)
        if traj is None:
            return False
        return self.execute_arm_plan(arm_id, traj)

    def move_arm_to_joint_values(self, arm_id, joint_values):
        traj = self.plan_arm_to_joint_values(arm_id, joint_values)
        if traj is None:
            return False
        return self.execute_arm_plan(arm_id, traj)

    # =====================================================
    # Planning / Execute - Gripper
    # =====================================================
    def plan_gripper_joint(self, arm_id, joint_value):
        group = self.gripper_groups[arm_id]
        joint_name = self.arm_cfg[arm_id]["gripper_joint"]

        rospy.logwarn("[%s] 夹爪目标: %s -> %.4f rad", arm_id, joint_name, joint_value)

        try:
            active_joints = group.get_active_joints()
        except Exception as e:
            rospy.logerr("[%s] 获取 gripper active_joints 失败: %s", arm_id, str(e))
            return None

        if joint_name not in active_joints:
            rospy.logerr("[%s] 夹爪组 %s 不包含关节 %s",
                         arm_id, self.arm_cfg[arm_id]["gripper_group_name"], joint_name)
            return None

        with self.plan_lock:
            group.set_start_state_to_current_state()

            try:
                group.set_joint_value_target({joint_name: joint_value})
            except Exception as e:
                rospy.logerr("[%s] set_joint_value_target(%s) 失败: %s", arm_id, joint_name, str(e))
                return None

            for i in range(3):
                rospy.logwarn("[%s] 第 %d 次夹爪规划...", arm_id, i + 1)
                try:
                    plan_result = group.plan()
                except Exception as e:
                    rospy.logerr("[%s] gripper plan() 异常: %s", arm_id, str(e))
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
                    rospy.logwarn("[%s] 夹爪规划成功", arm_id)
                    return traj

        rospy.logerr("[%s] 夹爪规划最终失败", arm_id)
        return None

    def execute_gripper_plan(self, arm_id, traj):
        group = self.gripper_groups[arm_id]

        rospy.logwarn("[%s] 等待执行锁(夹爪)...", arm_id)
        with self.exec_lock:
            rospy.logwarn("[%s] 已获得执行锁，开始执行夹爪轨迹...", arm_id)
            try:
                ok = group.execute(traj, wait=True)
                group.stop()
                rospy.logwarn("[%s] 夹爪执行结果 = %s", arm_id, str(ok))
                return ok
            except Exception as e:
                rospy.logerr("[%s] gripper execute() 异常: %s", arm_id, str(e))
                return False

    def move_gripper(self, arm_id, joint_value):
        traj = self.plan_gripper_joint(arm_id, joint_value)
        if traj is None:
            return False
        return self.execute_gripper_plan(arm_id, traj)

    def open_gripper_60deg(self, arm_id):
        rospy.logwarn("[%s] 准备张开夹爪60°", arm_id)
        return self.move_gripper(arm_id, self.gripper_open_angle)

    def close_gripper(self, arm_id):
        rospy.logwarn("[%s] 准备闭合夹爪", arm_id)
        return self.move_gripper(arm_id, self.gripper_close_angle)

    # =====================================================
    # V1 冲突判断
    # =====================================================
    def has_higher_priority(self, arm_a, arm_b):
        return self.priority_order[arm_a] < self.priority_order[arm_b]

    def target_conflict(self, arm_a, arm_b):
        xyz_a = self.get_target_xyz(arm_a)
        xyz_b = self.get_target_xyz(arm_b)

        if xyz_a is None or xyz_b is None:
            return False

        d = self.xy_distance(xyz_a, xyz_b)
        rospy.loginfo("[%s-%s] 抓取点XY距离 = %.4f", arm_a, arm_b, d)
        return d < self.conflict_xy_threshold

    def arm_blocks_me(self, me, other):
        if not self.has_higher_priority(other, me):
            return False

        if not self.target_conflict(me, other):
            return False

        other_state = self.get_arm_state(other)

        if other_state in ["waiting_for_grasp", "entering_grasp"]:
            rospy.logwarn("[%s] 被高优先级 %s 阻挡，原因: 状态=%s",
                          me, other, other_state)
            return True

        return False

    def can_enter_grasp_zone(self, arm_id):
        for other in ["arm1", "arm2", "arm3"]:
            if other == arm_id:
                continue
            if self.arm_blocks_me(arm_id, other):
                return False
        return True

    def wait_until_can_enter_grasp(self, arm_id):
        rospy.logwarn("[%s] 等待进入抓取区许可...", arm_id)
        self.set_arm_state(arm_id, "waiting_for_grasp")

        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(1.0 / self.wait_sleep)

        while not rospy.is_shutdown():
            if self.can_enter_grasp_zone(arm_id):
                rospy.logwarn("[%s] 已获得进入抓取区许可", arm_id)
                return True

            if rospy.Time.now().to_sec() - t0 > self.max_wait_time:
                rospy.logerr("[%s] 等待进入抓取区超时", arm_id)
                return False

            rate.sleep()

        return False

    # =====================================================
    # 调试函数
    # =====================================================
    def print_current_joint_values(self, arm_id):
        try:
            vals = self.arm_groups[arm_id].get_current_joint_values()
            rospy.logwarn("[%s] 当前关节角 = %s", arm_id, str(vals))
        except Exception as e:
            rospy.logerr("[%s] 获取当前关节角失败: %s", arm_id, str(e))

    # =====================================================
    # Arm task steps
    # =====================================================
    def move_to_safe_and_open(self, arm_id, target_xyz):
        # 这里仍沿用文档5的“目标点上方安全层”
        # 如果你想换成“启动初始姿态作为安全位”，我也能继续帮你改
        safe_pose = self.make_safe_pose(arm_id, target_xyz)

        self.set_arm_state(arm_id, "moving_safe")
        ok = self.move_arm_to_pose(arm_id, safe_pose)
        if not ok:
            rospy.logerr("[%s] 到安全层失败", arm_id)
            self.set_arm_state(arm_id, "failed")
            return False

        self.set_arm_state(arm_id, "safe_ready")
        rospy.logwarn("[%s] 已到达安全层", arm_id)

        ok = self.open_gripper_60deg(arm_id)
        if not ok:
            rospy.logerr("[%s] 安全层打开夹爪失败", arm_id)
            self.set_arm_state(arm_id, "failed")
            return False

        rospy.logwarn("[%s] 安全层夹爪已打开60°", arm_id)
        return True

    def move_direct_to_grasp_point(self, arm_id, target_xyz):
        grasp_xyz = [
            target_xyz[0],
            target_xyz[1],
            target_xyz[2] + self.final_target_offset_z
        ]

        if not self.target_is_reasonable(arm_id, grasp_xyz):
            rospy.logerr("[%s] 抓取点不合理", arm_id)
            self.set_arm_state(arm_id, "failed")
            return False

        grasp_pose = self.make_facing_pose(arm_id, grasp_xyz)

        self.set_arm_state(arm_id, "entering_grasp")
        ok = self.move_arm_to_pose(arm_id, grasp_pose)
        if not ok:
            rospy.logerr("[%s] 直接进入抓取点失败", arm_id)
            self.set_arm_state(arm_id, "failed")
            self.print_current_joint_values(arm_id)
            return False

        rospy.logwarn("[%s] 已到达抓取点", arm_id)
        return True

    # =====================================================
    # Main task
    # =====================================================
    def arm_task(self, arm_id):
        rospy.logwarn("[%s] ===== 任务线程启动 =====", arm_id)

        target_xyz = self.get_target_xyz(arm_id)
        rospy.logwarn("[%s] target_xyz = %s", arm_id, str(target_xyz))

        if target_xyz is None:
            rospy.logerr("[%s] 没有目标点", arm_id)
            self.set_arm_state(arm_id, "failed")
            return

        if not self.target_is_reasonable(arm_id, target_xyz):
            rospy.logerr("[%s] 目标点不合理", arm_id)
            self.set_arm_state(arm_id, "failed")
            return

        # 1. 先到安全层，并在安全层打开夹爪
        ok = self.move_to_safe_and_open(arm_id, target_xyz)
        if not ok:
            return

        rospy.sleep(0.3)

        # 2. 等待抓取区许可
        ok = self.wait_until_can_enter_grasp(arm_id)
        if not ok:
            self.set_arm_state(arm_id, "failed")
            return

        # 3. 直接进入抓取点（不做下探）
        ok = self.move_direct_to_grasp_point(arm_id, target_xyz)
        if not ok:
            return

        rospy.sleep(0.3)

        # 4. 闭合夹爪
        ok = self.close_gripper(arm_id)
        if not ok:
            rospy.logerr("[%s] 闭合夹爪失败", arm_id)
            self.set_arm_state(arm_id, "failed")
            return

        self.set_arm_state(arm_id, "grasped")
        rospy.logwarn("[%s] ===== 抓取完成 =====", arm_id)

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

        rospy.logwarn("===== THREE ARM HOIST V1 END =====")

if __name__ == "__main__":
    try:
        node = ThreeArmHoistV1()
        rospy.sleep(1.0)
        node.run()
    except rospy.ROSInterruptException:
        pass