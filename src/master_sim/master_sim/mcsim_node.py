import rclpy
from rclpy.node import Node
import numpy as np
from mcsimpy.simulator import CSAD_DP_6DOF
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Wrench
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class McsimNode(Node):
    def __init__(self):
        super().__init__("mcsim_node")
        self.get_logger().info("Simulation has started")
        
        self.odom_pub = self.create_publisher(Odometry, "measurement/odom", 10)
        self.wrench_sub = self.create_subscription(Wrench, "control/force/command", self.force_callback, 10)

        self.dt = 0.01
        self.counter = 0

        self.vessel = CSAD_DP_6DOF(dt=self.dt, method="RK4")
        self.tau = np.zeros(6)
        self.create_timer(self.dt, self.step)
        self.tf_broadcaster = TransformBroadcaster(self)

    def step(self):
        self.counter += 1
        self.vessel.integrate(0.0, 0.0, self.tau)

        eta = self.vessel.get_eta()
        nu = self.vessel.get_nu()
        q = Rotation.from_euler("xyz", eta[3:6]).as_quat()

        self.publish_odom(eta, nu, q)
        self.publish_transform(eta, q)

        if self.counter % 100 == 0:
            self.get_logger().info(f"t = {self.counter * self.dt:.1f} s, x = {eta[0]:.3f} m, y = {eta[1]:.3f} m, psi = {eta[5]:.3f} rad")
        


    def publish_odom(self, eta, nu, q):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link_ned"
        msg.pose.pose.position.x = float(eta[0])
        msg.pose.pose.position.y = float(eta[1])
        msg.pose.pose.position.z = float(eta[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = float(nu[0])
        msg.twist.twist.linear.y = float(nu[1])
        msg.twist.twist.linear.z = float(nu[2])
        msg.twist.twist.angular.x = float(nu[3])
        msg.twist.twist.angular.y = float(nu[4])
        msg.twist.twist.angular.z = float(nu[5])
        self.odom_pub.publish(msg)
        
    def publish_transform(self, eta, q):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link_ned"
        t.transform.translation.x = float(eta[0])
        t.transform.translation.y = float(eta[1])
        t.transform.translation.z = float(eta[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def force_callback(self, msg):
        self.tau[0] = msg.force.x
        self.tau[1] = msg.force.y
        self.tau[2] = msg.force.z
        self.tau[3] = msg.torque.x
        self.tau[4] = msg.torque.y
        self.tau[5] = msg.torque.z


def main(args=None):
    rclpy.init(args=args)
    node = McsimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()