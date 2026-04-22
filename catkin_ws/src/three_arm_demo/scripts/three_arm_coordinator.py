#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import copy
import math
import threading

import rospy
import moveit_commander

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from tf.transformations import quaternion_from_euler

class ThreeArmURDFAvoidanceScheduler(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("three_arm_urdf_scheduler_avoidance", anonymous=True)

        rospy.logwarn("===== THREE ARM URDF AVOIDANCE SCHEDULER START =====")
        rospy.logwarn("RUN FILE: %s", os.path.abspath(__file__))

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # =====================================================
        # 你的 URDF 对应配置
        # =====================================================
        self.world_frame = "world"

        self.cfg = {
            "arm1": {
                "group": "arm1_group",
                "tcp": "arm1_tcp",
                "gripper_joint": "arm1_J611",
                "topic": "/target/P",
                "priority": 3,
                "color": "P",
                "base_xyz": [0.3, 0.0, 0.0],
            },
            "arm2": {
                "group": "arm2_group",
                "tcp": "arm2_tcp",
                "gripper_joint": "arm2_J611",
                "topic": "/target/Y",
                "priority": 2,
                "color": "Y",
                "base_xyz": [-0.15, 0.2598076, 0.0],
            },
            "arm3": {
                "group": "arm3_group",
                "tcp": "arm3_tcp",
                "gripper_joint": "arm3_J611",
                "topic": "/target/G",
                "priority": 1,
                "color": "G",
                "base_xyz": [-0.15, -0.2598076, 0.0],
            }
        }

        # =====================================================
        # Move group
        # =====================================================
        self.arm_groups = {
            "arm1": moveit_commander.MoveGroupCommander(self.cfg["arm1"]["group"]),
            "arm2": moveit_commander.MoveGroupCommander(self.cfg["arm2"]["group"]),
            "arm3": moveit_commander.MoveGroupCommander(self.cfg["arm3"]["group"]),
        }

        # 假设你的 SRDF 中存在总夹爪组
        self.gripper_group = moveit_commander.MoveGroupCommander("all_grippers_group")

        # =====================================================
        # 执行锁
        # 避免多线程同时 execute 导致 MoveIt 打架
        # =====================================================
        self.exec_lock = threading.Lock()

        # =====================================================
        # 参数设置
        # =====================================================
        for arm_id, group in self.arm_groups.items():
            group.set_pose_reference_frame(self.world_frame)
            group.set_planning_time(5.0)
            group.set_num_planning_attempts(10)
            group.set_goal_position_tolerance(0.005)
            group.set_goal_orientation_tolerance(0.20)
            group.set_max_velocity_scaling_factor(0.12)
            group.set_max_acceleration_scaling_factor(0.12)
            try:
                group.set_planning_pipeline_id("ompl")
            except Exception:
                pass
            try:
                group.set_planner_id("RRTConnect")
            except Exception:
                pass
            group.allow_replanning(True)

        self.gripper_group.set_planning_time(2.0)
        self.gripper_group.set_num_planning_attempts(5)

        # =====================================================
        # 分层高度（按你的 URDF 保守设置）
        # =====================================================
        self.approach_height = 0.04
        self.transport_layer = 0.18
        self.safe_layer = 0.10
        self.place_approach_height = 0.04

        # =====================================================
        # 夹爪角度
        # 需要你根据实际再微调
        # =====================================================
        self.open_angle = -0.85
        self.close_angle = -0.18

        # 末端固定姿态，先用一个稳定可规划的姿态跑通闭环
        self.fixed_tcp_orientation = {
            "x": 0.01947845,
            "y": 0.00088976,
            "z": 0.99742708,
            "w": 0.06898569,
        }

        # =====================================================
        # 视觉坐标补偿
        # =====================================================
        self.auto_mm_to_m = True
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        # 末端抓取补偿
        self.grasp_z_offset = {
            "arm1": 0.02,
            "arm2": 0.02,
            "arm3": 0.02,
        }

        self.grasp_xy_offset = {
            "arm1": (0.0, 0.0),
            "arm2": (0.0, 0.0),
            "arm3": (0.0, 0.0),
        }

        # =====================================================
        # 放置点
        # 尽量放在各自相对独立区域
        # =====================================================
        self.place_points = {
            "arm1": [0.28, 0.18, 0.03],
            "arm2": [-0.02, 0.26, 0.03],
            "arm3": [-0.02, -0.26, 0.03],
        }

        # =====================================================
        # 冲突判定阈值
        # =====================================================
        self.threshold_now = 0.13
        self.threshold_target = 0.10
        self.threshold_transport = 0.16
        self.transport_z_diff_limit = 0.05

        # =====================================================
        # 状态
        # =====================================================
        self.targets = {
            "arm1": None,
            "arm2": None,
            "arm3": None,
        }

        self.arm_state = {
            "arm1": {
                "status": "IDLE",       # IDLE READY RUNNING WAIT SAFE DONE ERROR
                "phase": "NONE",        # OPEN_GRIPPER PRE_GRASP GRASP CLOSE_GRIPPER LIFT TRANSPORT PRE_PLACE PLACE RETURN DONE
                "current_xyz": None,
                "target_xyz": None,
                "place_xyz": self.place_points["arm1"],
                "pause": False,
                "task_done": False,
                "need_safe_lift": False,
            },
            "arm2": {
                "status": "IDLE",
                "phase": "NONE",
                "current_xyz": None,
                "target_xyz": None,
                "place_xyz": self.place_points["arm2"],
                "pause": False,
                "task_done": False,
                "need_safe_lift": False,
            },
            "arm3": {
                "status": "IDLE",
                "phase": "NONE",
                "current_xyz": None,
                "target_xyz": None,
                "place_xyz": self.place_points["arm3"],
                "pause": False,
                "task_done": False,
                "need_safe_lift": False,
            },
        }

        self.lock = threading.Lock()
        self.running = True

        # =====================================================
        # 订阅视觉目标
        # =====================================================
        rospy.Subscriber("/target/P", PointStamped, self.cb_arm1, queue_size=1)
        rospy.Subscriber("/target/Y", PointStamped, self.cb_arm2, queue_size=1)
        rospy.Subscriber("/target/G", PointStamped, self.cb_arm3, queue_size=1)

        rospy.sleep(1.0)

        self.print_group_info()
        self.check_urdf_consistency()

        rospy.sleep(0.5)
        self.setup_scene()
        rospy.sleep(1.0)

        rospy.loginfo("ThreeArmURDFAvoidanceScheduler 初始化完成")

    # =====================================================
    # 基础打印
    # =====================================================
    def print_group_info(self):
        rospy.loginfo("========== MoveIt Group Info ==========")
        for arm_id in ["arm1"]:
            try:
                joints = self.arm_groups[arm_id].get_active_joints()
                rospy.loginfo("%s active joints: %s", arm_id, joints)
            except Exception as e:
                rospy.logerr("%s group info 获取失败: %s", arm_id, str(e))

        try:
            joints_all = self.gripper_group.get_active_joints()
            rospy.loginfo("all_grippers_group active joints: %s", joints_all)
        except Exception as e:
            rospy.logwarn("all_grippers_group 获取失败: %s", str(e))

        rospy.loginfo("=======================================")

    def check_urdf_consistency(self):
        rospy.loginfo("========== URDF / MoveIt Consistency Check ==========")
        try:
            all_links = self.robot.get_link_names()
            all_joints = self.robot.get_joint_names()
        except Exception as e:
            rospy.logerr("RobotCommander 获取 link/joint 失败: %s", str(e))
            return

        for arm_id in ["arm1", "arm2", "arm3"]:
            tcp = self.cfg[arm_id]["tcp"]
            gj = self.cfg[arm_id]["gripper_joint"]

            if tcp in all_links:
                rospy.loginfo("[%s] tcp link OK: %s", arm_id, tcp)
            else:
                rospy.logerr("[%s] tcp link 不存在: %s", arm_id, tcp)

            if gj in all_joints:
                rospy.loginfo("[%s] gripper joint OK: %s", arm_id, gj)
            else:
                rospy.logerr("[%s] gripper joint 不存在: %s", arm_id, gj)

        rospy.loginfo("====================================================")

    # =====================================================
    # 场景
    # =====================================================
    def setup_scene(self):
        rospy.loginfo("添加桌面碰撞体...")

        self.scene.remove_world_object("table")
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
    # 视觉 callback
    # =====================================================
    def convert_point(self, msg):
        p = copy.deepcopy(msg)

        if self.auto_mm_to_m:
            if abs(p.point.x) > 5.0 or abs(p.point.y) > 5.0 or abs(p.point.z) > 5.0:
                p.point.x /= 1000.0
                p.point.y /= 1000.0
                p.point.z /= 1000.0

        p.point.x += self.offset_x
        p.point.y += self.offset_y
        p.point.z += self.offset_z
        return p

    def cb_arm1(self, msg):
        with self.lock:
            self.targets["arm1"] = self.convert_point(msg)

    def cb_arm2(self, msg):
        with self.lock:
            self.targets["arm2"] = self.convert_point(msg)

    def cb_arm3(self, msg):
        with self.lock:
            self.targets["arm3"] = self.convert_point(msg)

    # =====================================================
    # 工具函数
    # =====================================================
    def wait_for_all_targets(self):
        rospy.loginfo("等待三个目标点 /target/P /target/Y /target/G ...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                ok = all(self.targets[a] is not None for a in ["arm1", "arm2", "arm3"])
            if ok:
                rospy.loginfo("三个目标点均已收到")
                return
            rate.sleep()

    def get_target_xyz(self, arm_id):
        with self.lock:
            msg = self.targets[arm_id]
            if msg is None:
                return None

            x, y, z = msg.point.x, msg.point.y, msg.point.z

        dx, dy = self.grasp_xy_offset[arm_id]
        x += dx
        y += dy
        z += self.grasp_z_offset[arm_id]

        return [x, y, z]

    def get_current_tcp_pose(self, arm_id):
        tcp = self.cfg[arm_id]["tcp"]
        return self.arm_groups[arm_id].get_current_pose(tcp).pose

    def update_current_xyz(self, arm_id):
        try:
            pose = self.get_current_tcp_pose(arm_id)
            with self.lock:
                self.arm_state[arm_id]["current_xyz"] = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ]
        except Exception as e:
            rospy.logwarn("[%s] 更新当前TCP失败: %s", arm_id, str(e))

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

    def compute_yaw_to_target(self, arm_id, xyz):
        bx, by, _ = self.cfg[arm_id]["base_xyz"]
        x, y, _ = xyz
        return math.atan2(y - by, x - bx)

    def make_facing_pose(self, arm_id, xyz):
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        pose.orientation.x = self.fixed_tcp_orientation["x"]
        pose.orientation.y = self.fixed_tcp_orientation["y"]
        pose.orientation.z = self.fixed_tcp_orientation["z"]
        pose.orientation.w = self.fixed_tcp_orientation["w"]
        return pose

    def distance_xyz(self, p1, p2):
        if p1 is None or p2 is None:
            return 999.0
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dz = p1[2] - p2[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def target_is_reasonable(self, arm_id, xyz):
        x, y, z = xyz
        if abs(x) > 0.65 or abs(y) > 0.65:
            rospy.logwarn("[%s] x/y超范围: (%.4f, %.4f)", arm_id, x, y)
            return False
        if z < -0.03 or z > 0.35:
            rospy.logwarn("[%s] z超范围: %.4f", arm_id, z)
            return False

        bx, by, _ = self.cfg[arm_id]["base_xyz"]
        d = math.sqrt((x - bx) ** 2 + (y - by) ** 2)
        if d > 0.38:
            rospy.logwarn("[%s] 目标距基座过远: %.4f m", arm_id, d)
            return False

        return True

    def set_phase(self, arm_id, status, phase, target_xyz=None):
        with self.lock:
            self.arm_state[arm_id]["status"] = status
            self.arm_state[arm_id]["phase"] = phase
            self.arm_state[arm_id]["target_xyz"] = target_xyz

    # =====================================================
    # 单臂规划与执行
    # =====================================================
    def plan_single_arm(self, arm_id, target_pose):
        group = self.arm_groups[arm_id]
        tcp_link = self.cfg[arm_id]["tcp"]

        with self.exec_lock:
            group.set_start_state_to_current_state()
            group.set_position_target(
                [target_pose.position.x, target_pose.position.y, target_pose.position.z],
                end_effector_link=tcp_link,
            )

            for attempt in range(3):
                rospy.loginfo("[%s] 第%d次规划 -> tcp=%s", arm_id, attempt + 1, tcp_link)

                try:
                    plan_result = group.plan()
                except Exception as e:
                    rospy.logerr("[%s] plan()异常: %s", arm_id, str(e))
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
                    group.clear_pose_targets()
                    return traj

            group.clear_pose_targets()
            rospy.logerr("[%s] 规划失败", arm_id)
            return None

    def execute_single_arm(self, arm_id, traj):
        group = self.arm_groups[arm_id]
        with self.exec_lock:
            try:
                ok = group.execute(traj, wait=True)
                group.stop()
                group.clear_pose_targets()
                self.update_current_xyz(arm_id)
                return ok
            except Exception as e:
                rospy.logerr("[%s] execute()异常: %s", arm_id, str(e))
                return False

    def move_arm_to_pose(self, arm_id, pose):
        plan = self.plan_single_arm(arm_id, pose)
        if plan is None:
            rospy.logerr("[%s] move_arm_to_pose 规划失败", arm_id)
            return False
        ok = self.execute_single_arm(arm_id, plan)
        if not ok:
            rospy.logerr("[%s] move_arm_to_pose 执行失败", arm_id)
        return ok

    # =====================================================
    # 夹爪控制
    # =====================================================
    def set_single_gripper(self, arm_id, angle):
        joint_name = self.cfg[arm_id]["gripper_joint"]

        with self.exec_lock:
            try:
                active_joints = self.gripper_group.get_active_joints()
                rospy.loginfo("[%s] all_grippers_group joints = %s", arm_id, active_joints)

                if joint_name not in active_joints:
                    rospy.logerr("[%s] 夹爪关节 %s 不在 all_grippers_group 中", arm_id, joint_name)
                    return False

                self.gripper_group.set_joint_value_target({joint_name: angle})
                ok = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
                rospy.sleep(0.2)
                return ok

            except Exception as e:
                rospy.logerr("[%s] 夹爪控制失败: %s", arm_id, str(e))
                return False

    def open_gripper(self, arm_id):
        return self.set_single_gripper(arm_id, self.open_angle)

    def close_gripper(self, arm_id):
        return self.set_single_gripper(arm_id, self.close_angle)

    # =====================================================
    # 冲突检测
    # =====================================================
    def phase_is_dangerous(self, phase):
        return phase in [
            "PRE_GRASP", "GRASP", "LIFT", "TRANSPORT",
            "PRE_PLACE", "PLACE", "RETURN"
        ]

    def predict_conflict(self, arm_a, arm_b):
        with self.lock:
            state_a = copy.deepcopy(self.arm_state[arm_a])
            state_b = copy.deepcopy(self.arm_state[arm_b])

        if state_a["status"] in ["IDLE", "DONE", "ERROR"]:
            return False
        if state_b["status"] in ["IDLE", "DONE", "ERROR"]:
            return False

        if not self.phase_is_dangerous(state_a["phase"]):
            return False
        if not self.phase_is_dangerous(state_b["phase"]):
            return False

        pa = state_a["current_xyz"]
        pb = state_b["current_xyz"]

        d_now = self.distance_xyz(pa, pb)
        if d_now < self.threshold_now:
            return True

        ta = state_a["target_xyz"]
        tb = state_b["target_xyz"]

        d_target = self.distance_xyz(ta, tb)
        if d_target < self.threshold_target:
            return True

        if state_a["phase"] == "TRANSPORT" and state_b["phase"] == "TRANSPORT":
            if pa is not None and pb is not None:
                if abs(pa[2] - pb[2]) < self.transport_z_diff_limit and d_now < self.threshold_transport:
                    return True

        return False

    def higher_priority(self, arm_a, arm_b):
        pa = self.cfg[arm_a]["priority"]
        pb = self.cfg[arm_b]["priority"]
        return arm_a if pa >= pb else arm_b

    # =====================================================
    # 调度器
    # =====================================================
    def scheduler_loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.running:
            for arm_id in ["arm1", "arm2", "arm3"]:
                self.update_current_xyz(arm_id)

            pairs = [("arm1", "arm2"), ("arm1", "arm3"), ("arm2", "arm3")]

            for a, b in pairs:
                if self.predict_conflict(a, b):
                    hp = self.higher_priority(a, b)
                    lp = b if hp == a else a

                    with self.lock:
                        lp_state = self.arm_state[lp]
                        hp_state = self.arm_state[hp]

                        if lp_state["status"] not in ["WAIT", "SAFE", "DONE", "ERROR"]:
                            rospy.logwarn("检测到冲突: %s <-> %s, 高优先级=%s, 低优先级=%s",
                                          a, b, hp, lp)

                            if lp_state["phase"] in ["LIFT", "TRANSPORT", "PRE_PLACE", "PLACE", "RETURN"]:
                                lp_state["need_safe_lift"] = True
                                lp_state["status"] = "SAFE"
                                lp_state["pause"] = False
                            else:
                                lp_state["pause"] = True
                                lp_state["status"] = "WAIT"

                        if hp_state["status"] == "WAIT":
                            hp_state["status"] = "RUNNING"
                            hp_state["pause"] = False

            # 冲突解除恢复
            with self.lock:
                for arm_id in ["arm1", "arm2", "arm3"]:
                    st = self.arm_state[arm_id]
                    if st["status"] == "WAIT":
                        conflict_exists = False
                        for other in ["arm1", "arm2", "arm3"]:
                            if other == arm_id:
                                continue
                            if self.predict_conflict(arm_id, other):
                                conflict_exists = True
                                break
                        if not conflict_exists:
                            rospy.loginfo("[%s] 冲突解除，恢复运行", arm_id)
                            st["pause"] = False
                            st["status"] = "RUNNING"

            rate.sleep()

    # =====================================================
    # 等待/安全层
    # =====================================================
    def wait_if_paused(self, arm_id):
        while not rospy.is_shutdown():
            with self.lock:
                paused = self.arm_state[arm_id]["pause"]
                status = self.arm_state[arm_id]["status"]

            if not paused and status != "WAIT":
                return True

            rospy.sleep(0.1)

    def handle_safe_lift_if_needed(self, arm_id):
        with self.lock:
            need_safe = self.arm_state[arm_id]["need_safe_lift"]
            current_xyz = copy.deepcopy(self.arm_state[arm_id]["current_xyz"])

        if not need_safe:
            return True

        if current_xyz is None:
            return False

        rospy.logwarn("[%s] 执行安全层上升避障", arm_id)

        safe_xyz = [current_xyz[0], current_xyz[1], self.safe_layer]
        safe_pose = self.make_facing_pose(arm_id, safe_xyz)
        ok = self.move_arm_to_pose(arm_id, safe_pose)

        with self.lock:
            self.arm_state[arm_id]["need_safe_lift"] = False
            if ok:
                self.arm_state[arm_id]["status"] = "RUNNING"
            else:
                self.arm_state[arm_id]["status"] = "ERROR"

        return ok

    # =====================================================
    # 单臂任务流程
    # =====================================================
    def arm_task(self, arm_id):
        rospy.loginfo("[%s] 任务线程启动", arm_id)

        pick_xyz = self.get_target_xyz(arm_id)
        with self.lock:
            place_xyz = copy.deepcopy(self.arm_state[arm_id]["place_xyz"])

        if pick_xyz is None:
            rospy.logerr("[%s] 没有抓取目标", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        if not self.target_is_reasonable(arm_id, pick_xyz):
            rospy.logerr("[%s] 抓取目标不合理: %s", arm_id, str(pick_xyz))
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        # 0 打开夹爪
        self.set_phase(arm_id, "RUNNING", "OPEN_GRIPPER", pick_xyz)
        self.open_gripper(arm_id)
        rospy.sleep(0.2)

        # 1 预抓取
        pre_grasp_xyz = [pick_xyz[0], pick_xyz[1], pick_xyz[2] + self.approach_height]
        self.set_phase(arm_id, "RUNNING", "PRE_GRASP", pre_grasp_xyz)
        self.wait_if_paused(arm_id)
        if not self.handle_safe_lift_if_needed(arm_id):
            return

        pre_grasp_pose = self.make_facing_pose(arm_id, pre_grasp_xyz)
        if not self.move_arm_to_pose(arm_id, pre_grasp_pose):
            rospy.logerr("[%s] 预抓取失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        # 2 抓取
        self.set_phase(arm_id, "RUNNING", "GRASP", pick_xyz)
        self.wait_if_paused(arm_id)

        grasp_pose = self.make_facing_pose(arm_id, pick_xyz)
        if not self.move_arm_to_pose(arm_id, grasp_pose):
            rospy.logerr("[%s] 抓取位失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        rospy.sleep(0.2)

        # 3 关夹爪
        self.set_phase(arm_id, "RUNNING", "CLOSE_GRIPPER", pick_xyz)
        self.close_gripper(arm_id)
        rospy.sleep(0.4)

        # 4 抬升到运输层
        lift_xyz = [pick_xyz[0], pick_xyz[1], self.transport_layer]
        self.set_phase(arm_id, "RUNNING", "LIFT", lift_xyz)
        self.wait_if_paused(arm_id)

        lift_pose = self.make_facing_pose(arm_id, lift_xyz)
        if not self.move_arm_to_pose(arm_id, lift_pose):
            rospy.logerr("[%s] 抬升失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        # 5 必要时进入安全层
        if not self.handle_safe_lift_if_needed(arm_id):
            rospy.logerr("[%s] 安全层上升失败", arm_id)
            return

        # 6 平移运输
        transport_xyz = [place_xyz[0], place_xyz[1], self.transport_layer]
        self.set_phase(arm_id, "RUNNING", "TRANSPORT", transport_xyz)
        self.wait_if_paused(arm_id)

        transport_pose = self.make_facing_pose(arm_id, transport_xyz)
        if not self.move_arm_to_pose(arm_id, transport_pose):
            rospy.logerr("[%s] 运输平移失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        # 7 必要时进入安全层
        if not self.handle_safe_lift_if_needed(arm_id):
            rospy.logerr("[%s] 第二次安全层避障失败", arm_id)
            return

        # 8 放置预备位
        place_pre_xyz = [place_xyz[0], place_xyz[1], place_xyz[2] + self.place_approach_height]
        self.set_phase(arm_id, "RUNNING", "PRE_PLACE", place_pre_xyz)
        self.wait_if_paused(arm_id)

        place_pre_pose = self.make_facing_pose(arm_id, place_pre_xyz)
        if not self.move_arm_to_pose(arm_id, place_pre_pose):
            rospy.logerr("[%s] 放置预备位失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        # 9 放置
        self.set_phase(arm_id, "RUNNING", "PLACE", place_xyz)
        self.wait_if_paused(arm_id)

        place_pose = self.make_facing_pose(arm_id, place_xyz)
        if not self.move_arm_to_pose(arm_id, place_pose):
            rospy.logerr("[%s] 放置失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        rospy.sleep(0.2)

        # 10 打开夹爪
        self.set_phase(arm_id, "RUNNING", "OPEN_AT_PLACE", place_xyz)
        self.open_gripper(arm_id)
        rospy.sleep(0.3)

        # 11 回到运输层
        return_xyz = [place_xyz[0], place_xyz[1], self.transport_layer]
        self.set_phase(arm_id, "RUNNING", "RETURN", return_xyz)
        self.wait_if_paused(arm_id)

        return_pose = self.make_facing_pose(arm_id, return_xyz)
        if not self.move_arm_to_pose(arm_id, return_pose):
            rospy.logerr("[%s] 返回运输层失败", arm_id)
            with self.lock:
                self.arm_state[arm_id]["status"] = "ERROR"
            return

        with self.lock:
            self.arm_state[arm_id]["status"] = "DONE"
            self.arm_state[arm_id]["phase"] = "DONE"
            self.arm_state[arm_id]["task_done"] = True

        rospy.loginfo("[%s] 任务完成", arm_id)

    # =====================================================
    # 监控打印
    # =====================================================
    def print_states_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and self.running:
            with self.lock:
                for arm_id in ["arm1", "arm2", "arm3"]:
                    st = self.arm_state[arm_id]
                    rospy.loginfo("[%s] status=%s, phase=%s, current=%s, target=%s",
                                  arm_id,
                                  st["status"],
                                  st["phase"],
                                  str(st["current_xyz"]),
                                  str(st["target_xyz"]))
            rate.sleep()

    # =====================================================
    # 主流程
    # =====================================================
    def run(self):
        self.wait_for_all_targets()

        for arm_id in ["arm1", "arm2", "arm3"]:
            target_xyz = self.get_target_xyz(arm_id)
            with self.lock:
                self.arm_state[arm_id]["target_xyz"] = target_xyz
                self.arm_state[arm_id]["status"] = "READY"

        scheduler_thread = threading.Thread(target=self.scheduler_loop)
        scheduler_thread.daemon = True
        scheduler_thread.start()

        monitor_thread = threading.Thread(target=self.print_states_loop)
        monitor_thread.daemon = True
        monitor_thread.start()

        threads = []
        for arm_id in ["arm1", "arm2", "arm3"]:
            t = threading.Thread(target=self.arm_task, args=(arm_id,))
            t.daemon = True
            t.start()
            threads.append(t)

        for t in threads:
            t.join()

        self.running = False
        rospy.loginfo("所有机械臂任务执行结束")

if __name__ == "__main__":
    try:
        node = ThreeArmURDFAvoidanceScheduler()
        rospy.sleep(1.0)
        node.run()
    except rospy.ROSInterruptException:
        pass
