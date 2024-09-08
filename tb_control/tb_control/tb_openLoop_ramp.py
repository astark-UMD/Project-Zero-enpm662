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
        
# The system is designed to ramp up in speed for the first third of time traveled,
# maintain speed for the second third of time traveled, and ramp down speed for the last
# third of time traveled.
distance = 1 # meters
time_to_travel = 10 # seconds
max_velocity = distance/time_to_travel*3/2 # m/s
current_velocity = 0

def main():
    print('Hi from tb_control.')
    
    rclpy.init()
    
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('tb_openLoop')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    try:
        start_time = time.time()
        
        print("tb_control: accelerating")
        # Ramp up speed
        while(time.time() < start_time + time_to_travel/3):
            twist = Twist()
            twist.linear.x = max_velocity/(time_to_travel/3)*(time.time() - start_time)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            pub.publish(twist)
    
        print("tb_control: steady speed")
        # maintain speed
        while(time.time() < start_time + 2*time_to_travel/3):
            twist = Twist()
            twist.linear.x = max_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            pub.publish(twist)
            
        print("tb_control: coming to a stop")
        # slow down speed
        while(time.time() < start_time + time_to_travel):
            twist = Twist()
            twist.linear.x = max_velocity - max_velocity/(time_to_travel/3)*(time.time() - (start_time + (2/3)*time_to_travel))
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            pub.publish(twist)
            
            
            
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
