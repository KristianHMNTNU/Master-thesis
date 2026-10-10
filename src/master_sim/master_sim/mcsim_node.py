import rclpy
from rclpy.node import Node
import numpy as np
from mcsimpy.simulator import CSAD_DP_6DOF, CSADMan3DOF, RVG_DP_6DOF, GunnerusManeuvering3DoF
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Wrench, WrenchStamped, TransformStamped
from tf2_ros import TransformBroadcaster

VESSELS = {
    "CSAD_DP_6DOF": CSAD_DP_6DOF,
    "CSADMan3DOF": CSADMan3DOF,
    "RVG_DP_6DOF": RVG_DP_6DOF,
    "GunnerusManeuvering3DoF": GunnerusManeuvering3DoF,
}



class McsimNode(Node):
    def __init__(self):
        super().__init__("mcsim_node")

        self.declare_parameter("vessel", "CSAD_DP_6DOF")
        self.declare_parameter("dt", 0.01)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("current_speed", 0.0)
        self.declare_parameter("current_direction", 0.0)

        vessel_name = self.get_parameter("vessel").value
        self.dt = self.get_parameter("dt").value
        x0, y0, psi0 = self.get_parameter("initial_pose").value

        self.vessel = VESSELS[vessel_name](dt=self.dt, method="RK4")
        self.dof = self.vessel._dof
        if self.dof == 3:
            self.vessel.set_eta(np.array([x0, y0, psi0]))
        else:
            self.vessel.set_eta(np.array([x0, y0, 0.0, 0.0, 0.0, psi0]))

        self.get_logger().info(f"Simulating {vessel_name} ({self.dof} DOF), dt = {self.dt} s")

        self.tau = np.zeros(6)
        self.counter = 0
        self.odom_pub = self.create_publisher(Odometry, "measurement/odom", 10)
        self.tau_pub = self.create_publisher(WrenchStamped, "simulator/tau", 10)
        self.wrench_sub = self.create_subscription(Wrench, "control/force/command", self.force_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(self.dt, self.step)
        self.log_every = round(1.0 / self.dt)

    def step(self):
        self.counter += 1

        if self.dof == 3:
            tau = self.tau[[0, 1, 5]]
        else:
            tau = self.tau


        Uc = self.get_parameter("current_speed").value
        beta_c = self.get_parameter("current_direction").value
        self.vessel.integrate(Uc, beta_c, tau)

        eta = self.vessel.get_eta()
        nu = self.vessel.get_nu()

        if self.dof == 3:
            eta = np.array([eta[0], eta[1], 0.0, 0.0, 0.0, eta[2]])
            nu = np.array([nu[0], nu[1], 0.0, 0.0, 0.0, nu[2]])

        q = Rotation.from_euler("xyz", eta[3:6]).as_quat()

        self.publish_odom(eta, nu, q)
        self.publish_transform(eta, q)
        self.publish_tau()

        if self.counter % self.log_every == 0:
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

    def publish_tau(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_ned"
        msg.wrench.force.x = float(self.tau[0])
        msg.wrench.force.y = float(self.tau[1])
        msg.wrench.force.z = float(self.tau[2])
        msg.wrench.torque.x = float(self.tau[3])
        msg.wrench.torque.y = float(self.tau[4])
        msg.wrench.torque.z = float(self.tau[5])
        self.tau_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = McsimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()