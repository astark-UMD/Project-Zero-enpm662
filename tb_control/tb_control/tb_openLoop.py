import rclpy
from rclpy.node import Node

import time

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class tb_openLoop_publisher(Node):
    def __init__(self):
        qos = QoSProfile(depth=10)
        node = rclpy.create_node('tb_openLoop')
        self.pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    def publish_velocity(self, velocity):
        twist = Twist()
        twist.linear.x = velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        

distance = 1 # meters
time_to_travel = 10 # seconds
velocity = distance/time_to_travel # m/s

def main():
    print('Hi from tb_control.')
    
    rclpy.init()
    
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('tb_openLoop')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    try:
        start_time = time.time()
        print("tb_control: moving for # time")
        while(time.time() < start_time + time_to_travel):
            twist = Twist()
            twist.linear.x = .1
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            pub.publish(twist)
            
        print("tb_control: # seconds have passed")
            
    except Exception as e:
        print(e)
        
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)
        print("tb_control: finished.")
    
    # tb_openLoop = tb_openLoop_publisher()
    # tb_openLoop.publish_velocity(velocity)
    # time.sleep(time_to_travel)
    # tb_openLoop.publish_velocity(0.0)

if __name__ == '__main__':
    main()
