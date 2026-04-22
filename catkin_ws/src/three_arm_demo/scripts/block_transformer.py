#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

CENTER_U = 139
CENTER_V = 126
PIXEL_TO_METER = 0.004545   # 米/像素
GRIP_Z = 0.01               # 抓取高度（米）

COLOR_MAP = {
    'P': (0.6, 0.0, 1.0),
    'Y': (1.0, 1.0, 0.0),
    'G': (0.0, 1.0, 0.0),
}

# 【修复处】：在全局创建 Publisher，避免循环内创建导致丢包
publishers_point = {}
publishers_marker = {}

def pixel_to_world(u, v):
    dx = (u - CENTER_U) * PIXEL_TO_METER
    dy = -(v - CENTER_V) * PIXEL_TO_METER
    return dx, dy, GRIP_Z

def callback(msg):
    data = msg.data.strip().split(';')
    for item in data:
        if not item: continue
        parts = item.split(',')
        if len(parts) != 3: continue
        
        color, u_str, v_str = parts
        if color not in ['P', 'Y', 'G']: continue
        
        try:
            u = int(u_str)
            v = int(v_str)
        except ValueError: continue
        
        x, y, z = pixel_to_world(u, v)

        # 1. 发布真实坐标
        point = PointStamped()
        point.header.frame_id = "world"
        point.header.stamp = rospy.Time.now()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        publishers_point[color].publish(point)

        # 2. 发布 Rviz 可视化方块
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "blocks"
        marker.id = ord(color)  
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02   # 2cm
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        r, g, b = COLOR_MAP[color]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        publishers_marker[color].publish(marker)

        rospy.loginfo(f"[{color}] 发送目标 -> 真实坐标: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

if __name__ == '__main__':
    rospy.init_node('block_transformer')
    
    # 提前初始化所有的 Publisher
    for c in ['P', 'Y', 'G']:
        publishers_point[c] = rospy.Publisher(f'/target/{c}', PointStamped, queue_size=5,latch=True)
        publishers_marker[c] = rospy.Publisher(f'/target_marker/{c}', Marker, queue_size=5,latch=True)
        
    rospy.Subscriber('/openmv/blocks', String, callback)
    rospy.loginfo("坐标转换服务已启动...")
    rospy.spin()