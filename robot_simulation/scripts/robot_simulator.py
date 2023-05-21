#!/usr/bin/env python



import rospy
import control
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistWithCovariance, PoseWithCovariance
from nav_msgs.msg import Odometry
from robot_simulation.msg import Encoder
from tf.transformations import *
from math import sin, cos


class mot_mod:
    def __init__(self):
        k = 1
        T = 0.1
        W = control.tf(k, [T, 1])
        self.sys = control.LinearIOSystem(W)
        self.x = [[0, 0]]
        self.w_target = 0
        self.enc = 0

    def step(self, w_target, dt, cur_t, prev_t):
        a, w, self.x = control.input_output_response(self.sys, [prev_t, cur_t], [self.w_target, w_target], self.x[0][1], return_x=True)
        # разница между показаниями энкодера на текущей и предыдущей итерациях 
        d_enc = int(w[1] * dt / 2 / 3.1415926535 * 4096)
        
        # угловая скрорость колеса на данный момент
        cur_w = d_enc * 2 * 3.1415926535 / dt / 4096

        self.enc += d_enc
        self.w_target = w_target
        return cur_w


class motion_control:

    def __init__(self):

        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback) 
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()
        self.l_mot = mot_mod()
        self.r_mot = mot_mod()
        self.rot = 0
        self.L = 0.287
        self.r = 0.033
        self.prev_t = rospy.get_time()


    def callback(self, msg):
        rospy.loginfo("Linear Component: [%f]"%(msg.linear.x))
        rospy.loginfo("Angular Component: [%f]"%(msg.angular.z))

        
        V = msg.linear.x 
        Om = msg.angular.z
        
        # Найдем требуемые угловые скорости колес
    
        lw = (V - 0.5*Om*self.L)/self.r
        rw = (V + 0.5*Om*self.L)/self.r
        
        cur_t = rospy.get_time()
        dt = cur_t - self.prev_t
        
        # Вычислим текущие угловые скорости колес

        cur_spd_lw = self.l_mot.step(lw, dt, cur_t, self.prev_t)
        cur_spd_rw = self.r_mot.step(rw, dt, cur_t, self.prev_t)
        
        # Вычислим текущую линейную и угловую скорости робота, а также изменение его координаты и угол поворота

        V = 0.5*self.r * (cur_spd_lw + cur_spd_rw)
        Om = self.r/self.L * (cur_spd_rw - cur_spd_lw)
        
        x = V * cos(self.rot) * dt
        y = V * sin(self.rot) * dt
        self.rot += Om*dt

        
        self.pose.pose.position.x += x
        self.pose.pose.position.y += y

        q = quaternion_from_euler(0, 0, self.rot)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

        self.twist.twist.linear.x = V * cos(self.rot)
        self.twist.twist.linear.y = V * sin(self.rot)
        self.twist.twist.angular.z = Om
        self.prev_t = cur_t



enc_pub = rospy.Publisher("/enc", Encoder, queue_size=10)
od_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

# публикует показания левого и правого энкодеров
def talker_enc(l_enc, r_enc):

    msg = Encoder()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "enc"
    msg.left_encoder=l_enc 
    msg.right_encoder=r_enc 

    rospy.loginfo(msg)
    enc_pub.publish(msg)

def talker_odom(pose, twist):

    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "odom"# "base_link" мб
    msg.pose = pose 
    msg.twist = twist 

    rospy.loginfo(msg)
    od_pub.publish(msg) 


if __name__ == '__main__':
    rospy.init_node("Robot")
    m_c = motion_control()
    rt = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        talker_enc(m_c.l_mot.enc, m_c.r_mot.enc)
        talker_odom(m_c.pose, m_c.twist)
        rt.sleep()       

