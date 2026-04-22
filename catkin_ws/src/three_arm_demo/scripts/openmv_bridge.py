#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String

class OpenMVBridge:
    def __init__(self):
        rospy.init_node('openmv_bridge', anonymous=True)
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud = rospy.get_param('~baud', 115200)
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            rospy.loginfo(f"Connected to {self.port} at {self.baud}")
        except Exception as e:
            rospy.logerr(f"Failed to open serial: {e}")
            rospy.signal_shutdown("Serial error")
            return
        self.pub = rospy.Publisher('/openmv/blocks', String, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.pub.publish(String(line))
                    rospy.loginfo(f"Received: {line}")
            rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        node = OpenMVBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass