import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rclpy import qos

class BotCommander(Node): 
  def  __init__(self):
    super().__init__("bot_commander_node")
    obstacle_warn_topic_name = "/obstacle_ahead"
    user_cmd_topic_name = "/cmd_vel"
    bot_cmd_topic_name = "bot/cmd_vel"
    self.distance_threshold_ = 2.0

    self.bot_twist_ = Twist()
    self.user_twist_ = Twist()

    self.is_obstacle_present_ = False

    self.bot_cmd_pub_  = self.create_publisher(Twist, bot_cmd_topic_name, 10)

    self.warning_sub_ = self.create_subscription(Bool, obstacle_warn_topic_name, self.warningCallback, qos.qos_profile_sensor_data)
    self.user_cmd_sub_ = self.create_subscription(Twist, user_cmd_topic_name, self.userTwistCallback, qos.qos_profile_system_default)
    
    self.cmd_timer_ = self.create_timer(0.05, self.timerCallBack)

  def userTwistCallback(self, msg:Twist):
    self.user_twist_ = msg
  
  def warningCallback(self, msg:Bool):
    self.is_obstacle_present_ = msg.data
      
  def timerCallBack(self):
      if self.is_obstacle_present_:
        # If Obstacle present, set the command such that the rover makes a safe circular turn in front
        # of the wall. The turning radius should be less than the threshold distance to avoid touching
        # touching the wall.
        self.bot_twist_.linear.x = 0.05
        self.bot_twist_.angular.z = self.bot_twist_.linear.x /(0.5*self.distance_threshold_)
        self.bot_cmd_pub_.publish(self.bot_twist_)
      else:
        self.bot_cmd_pub_.publish(self.user_twist_)
    
###########################################################

def main(args=None):
    rclpy.init(args=args)

    commander_node = BotCommander()
    
    rclpy.spin(commander_node)

    commander_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
