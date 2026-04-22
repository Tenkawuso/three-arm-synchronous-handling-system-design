#!/usr/bin/env python3
import rospy
import serial
import struct
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
class OpenMVImageReceiver:
    def __init__(self):
        rospy.init_node('openmv_image_receiver')
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud = rospy.get_param('~baud', 115200)
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/openmv/image', Image, queue_size=1)
        self.coord_pub = rospy.Publisher('/openmv/blocks', String, queue_size=10)
        self.buffer = b''
        rospy.loginfo("Image receiver started")

    def run(self):
        while not rospy.is_shutdown():
            try:
                # 读取4字节长度
                length_data = self.ser.read(4)
                if len(length_data) < 4:
                    continue
                img_len = struct.unpack('<I', length_data)[0]
                # 读取图像数据
                jpeg_data = self.ser.read(img_len)
                if len(jpeg_data) < img_len:
                    continue
                # 读取换行和坐标字符串（可能有多行，但简单处理）
                self.ser.readline()  # 跳过换行
                coord_line = self.ser.readline().decode('utf-8').strip()
                # 发布图像
                np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_img is not None:
                    ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    self.image_pub.publish(ros_img)
                # 发布坐标
                if coord_line and coord_line != "NONE":
                    self.coord_pub.publish(String(coord_line))
                    rospy.loginfo(f"Coord: {coord_line}")
            except Exception as e:
                rospy.logerr(f"Error: {e}")
                break
        self.ser.close()

if __name__ == '__main__':
    node = OpenMVImageReceiver()
    node.run()