#!python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point32, Twist
from sensor_msgs.msg import LaserScan, PointCloud

STATE_STOP= 0
STATE_MOVE= 1
STATE_LEFT= 2
STATE_RIGHT= 3

class ReactiveMoveNode(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.obstacles_publisher = self.create_publisher(PointCloud, '/obstacles', 10)
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.obstacles= PointCloud()
        self.velocity_publisher= self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.control_publisher = self.create_publisher(PointCloud, '/control', 10)
        self.create_timer(0.02, self.control_callback)
        self.state= STATE_STOP
    
    def scan_callback(self, scanMsg):
        self.obstacles= PointCloud()
        self.obstacles.header= scanMsg.header
        angle= scanMsg.angle_min
        zero= (float)(0)
        for aDistance in scanMsg.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= Point32()
                aPoint.x= (float)(math.cos(angle) * aDistance)
                aPoint.y= (float)(math.sin( angle ) * aDistance)
                aPoint.z= zero
                self.obstacles.points.append( aPoint )
            angle+= scanMsg.angle_increment
        
        self.obstacles_publisher.publish( self.obstacles )

    def control_callback(self):
        # Security
        if len(self.obstacles.points) == 0 :
            print("Stop")
            self.state= STATE_STOP
            self.velocity_publisher.publish(Twist())
            return

        # Scan Interpretation
        focus= PointCloud()
        focus.header= self.obstacles.header

        obstacleLeft= False
        obstacleRight= False
        for obs in self.obstacles.points :
            if 0.01 < obs.x and obs.x < 0.4 and -0.4 < obs.y and obs.y < 0.4 :
                if obs.y < 0.0 :
                    obstacleRight= True
                else :
                    obstacleLeft= True
                focus.points.append( obs )
        
        # State estimation 
        self.control_publisher.publish( focus )
        if obstacleLeft :
            if self.state in [ STATE_STOP, STATE_MOVE ] :
                print("go right!")
                self.state= STATE_RIGHT
            print("go right!")
        elif obstacleRight :
            if self.state in [ STATE_STOP, STATE_MOVE ] :
                print("go left!")
                self.state= STATE_LEFT
            print("go left!")
        elif self.state != STATE_MOVE :
               print("move move move")
               self.state= STATE_MOVE
        
        #Control
        velo = Twist()
        if self.state == STATE_RIGHT :
            velo.angular.z= (float)(-1.0)
        elif self.state == STATE_LEFT :
            velo.angular.z= (float)(1.0)
        elif self.state == STATE_MOVE :
            velo.linear.x= (float)(0.4)

        self.velocity_publisher.publish(velo)   

def move(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Initialize ScanInterperter
    scanInterpret= ReactiveMoveNode()
    
    # infinite Loop
    rclpy.spin(scanInterpret)
    
    # Clean end
    scanInterpret.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    move()
