#!/usr/bin/env python3
"""Week 2 - Solution 3: Full EKF Fusion Node (Complete Implementation)"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Point, Pose, Quaternion, PoseWithCovariance, TwistWithCovariance, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class EKFFusionNode(Node):
    def __init__(self):
        super().__init__("ekf_fusion_node")
        self.declare_parameter("process_noise_q", 0.01)
        self.declare_parameter("measurement_noise_r", 0.1)
        self.q_noise = self.get_parameter("process_noise_q").value
        self.r_noise = self.get_parameter("measurement_noise_r").value
        self.state = np.zeros(10)
        self.state[6] = 1.0
        self.P = np.eye(10) * 0.1
        self.imu_sub = self.create_subscription(Imu, "/imu0", self.imu_cb, 10)
        self.pos_sub = self.create_subscription(PointStamped, "/leica/position", self.pos_cb, 10)
        self.fused_pub = self.create_publisher(Odometry, "/ekf_fused", 10)
        self.cov_pub = self.create_publisher(Float64MultiArray, "/ekf_covariance", 10)
        self.last_time = None
        self.get_logger().info("EKF Fusion Node started")

    def qm(self, q, r):
        return np.array([q[0]*r[0]-q[1]*r[1]-q[2]*r[2]-q[3]*r[3],
                         q[0]*r[1]+q[1]*r[0]+q[2]*r[3]-q[3]*r[2],
                         q[0]*r[2]-q[1]*r[3]+q[2]*r[0]+q[3]*r[1],
                         q[0]*r[3]+q[1]*r[2]-q[2]*r[1]+q[3]*r[0]])

    def q2r(self, q):
        w,x,y,z = q
        return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                         [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                         [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.last_time is None: self.last_time=t; return
        dt = t - self.last_time; self.last_time = t
        if dt<=0 or dt>1.0: return
        a = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        g = np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        q = self.state[6:10]
        R = self.q2r(q)
        aw = R @ a - np.array([0,0,9.81])
        self.state[0:3] += self.state[3:6]*dt + 0.5*aw*dt**2
        self.state[3:6] += aw*dt
        oq = np.array([0.0, g[0], g[1], g[2]])
        qd = 0.5*self.qm(q, oq)
        self.state[6:10] += qd*dt
        self.state[6:10] /= np.linalg.norm(self.state[6:10])
        F = np.eye(10); F[0:3,3:6] = np.eye(3)*dt
        Q = np.eye(10)*self.q_noise*dt
        self.P = F@self.P@F.T + Q
        self.publish_state(msg.header)

    def pos_cb(self, msg):
        z = np.array([msg.point.x, msg.point.y, msg.point.z])
        H = np.zeros((3,10)); H[0,0]=H[1,1]=H[2,2]=1.0
        R = np.eye(3)*self.r_noise
        y = z - H@self.state
        S = H@self.P@H.T + R
        K = self.P@H.T@np.linalg.inv(S)
        self.state += K@y
        self.P = (np.eye(10)-K@H)@self.P
        qn = np.linalg.norm(self.state[6:10])
        if qn>0: self.state[6:10] /= qn
        self.publish_state(msg.header)

    def publish_state(self, header):
        s = self.state
        odom = Odometry()
        odom.header.stamp = header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose = PoseWithCovariance(
            pose=Pose(position=Point(x=float(s[0]),y=float(s[1]),z=float(s[2])),
                      orientation=Quaternion(w=float(s[6]),x=float(s[7]),y=float(s[8]),z=float(s[9]))),
            covariance=self.P[:6,:6].flatten().tolist())
        self.fused_pub.publish(odom)
        cm = Float64MultiArray()
        cm.data = self.P.flatten().tolist()
        self.cov_pub.publish(cm)


def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
