# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TextToCmdVel(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # издатель (publisher), сообщения типа Twist,команды управления движением публикуются в /turtle1/cmd_vel. 10 - размер очереди сообщений.
        self.cmd_text_subscription = self.create_subscription(String, 'cmd_text', self.cmd_text_callback, 10) 
        self.twist = Twist()

    def cmd_text_callback(self, msg):
        cmd = msg.data.lower()
        
        if cmd == 'turn_right':
            self.twist.angular.z = -1.5
        elif cmd == 'turn_left':
            self.twist.angular.z = 1.5
        elif cmd == 'move_forward':
            self.twist.linear.x = 1.0
        elif cmd == 'move_backward':
            self.twist.linear.x = -1.0

        self.cmd_vel_publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

