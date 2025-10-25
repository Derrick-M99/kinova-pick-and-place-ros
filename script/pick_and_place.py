import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PickAndPlace:
    def __init__(self):
        rospy.init_node("pick_and_place")
        
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        rospy.loginfo("Using robot_name: " + self.robot_name)
        
        self.joint_state_sub = rospy.Subscriber(
            f"/{self.robot_name}/joint_states", JointState, self.cb_joint_state
        )
        
        self.trajectory_pub = rospy.Publisher(
            f"/{self.robot_name}/gen3_joint_trajectory_controller/command", JointTrajectory, queue_size=10
        )
        
        self.joint_angles = None
        self.home_position = None  

        #PICK AND PLACE POSITIONS
        self.pick_position = np.array([0.3, -0.2, 0.2]) 
        self.place_position = np.array([0.6, 0.3, 0.4])  

        self.joint_limits = {
            "min": np.array([-np.inf, -2.25, -np.inf, -2.58, -np.inf, -2.1, -np.inf]),  
            "max": np.array([ np.inf,  2.25,  np.inf,  2.58,  np.inf,  2.1,  np.inf])   
        }

    def cb_joint_state(self, joint_state):
        self.joint_angles = joint_state.position

    def clamp_joint_angles(self, q):
        return np.clip(q, self.joint_limits["min"], self.joint_limits["max"])

    def forward_kinematics(self, joint_angles):
        dh_params = [
            [np.pi, 0.0, 0.0, 0],
            [np.pi / 2, 0.0, -0.2848, joint_angles[0]],
            [np.pi / 2, 0.0, -0.0118, joint_angles[1] + np.pi],
            [np.pi / 2, 0.0, -0.4208, joint_angles[2] + np.pi],
            [np.pi / 2, 0.0, -0.0128, joint_angles[3] + np.pi],
            [np.pi / 2, 0.0, -0.3143, joint_angles[4] + np.pi],
            [np.pi / 2, 0.0, 0.0, joint_angles[5] + np.pi],
            [np.pi, 0.0, -0.1674, joint_angles[6] + np.pi],
        ]
        T = np.eye(4)
        for params in dh_params:
            T = np.dot(T, self.dh_transform(*params))
        position = T[:3, 3]
        orientation = T[:3, :3]
        return position, orientation

    def dh_transform(self, alpha, a, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def inverse_kinematics(self, target_position, initial_joint_angles):
        rospy.loginfo(f"Attempting IK for target position: {target_position}")
        q = initial_joint_angles
        max_iterations = 1000
        for iteration in range(max_iterations):
            current_position, _ = self.forward_kinematics(q)
            position_error = target_position - current_position
            
            if np.all(np.abs(position_error) < 1e-6):
                rospy.loginfo(f"IK converged in {iteration + 1} iterations.")
                return q

            jacobian = np.zeros((3, 7))
            delta = 1e-6
            for i in range(7):
                q_plus = np.copy(q)
                q_plus[i] += delta
                position_plus, _ = self.forward_kinematics(q_plus)
                jacobian[:, i] = (position_plus - current_position) / delta

            q += np.dot(np.linalg.pinv(jacobian), position_error)
            q = self.clamp_joint_angles(q)

        rospy.logerr("Inverse kinematics did not converge.")
        return None

    def radians_to_degrees(self, radians):
        return [np.degrees(angle) for angle in radians]

    def log_joint_info(self, joint_angles, description):
        rospy.loginfo(f"{description} Joint Angles (Radians): {joint_angles}")
        rospy.loginfo(f"{description} Joint Angles (Degrees): {self.radians_to_degrees(joint_angles)}")

    def log_position_orientation(self, position, orientation, description):
        rospy.loginfo(f"{description} Position of the End Effector: {position}")
        rospy.loginfo(f"{description} Orientation of the End Effector: {orientation}")

    def send_joint_trajectory(self, joint_angles, description=""):
        rospy.loginfo(f"Sending {description} joint trajectory: {joint_angles}")
        traj = JointTrajectory()
        traj.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(2.0) 
        traj.points.append(point)
        self.trajectory_pub.publish(traj)
        rospy.loginfo(f"{description} joint trajectory sent.")

    def compute_error_matrix(self, target_position, actual_position, target_orientation, actual_orientation, description):
        # Compute position error
        position_error = target_position - actual_position

        # Compute orientation error as a rotational matrix
        orientation_error = np.dot(target_orientation, actual_orientation.T)

        # Construct the 4x4 error matrix
        error_matrix = np.eye(4)
        error_matrix[:3, :3] = orientation_error  
        error_matrix[:3, 3] = position_error      

        rospy.loginfo(f"\n{description} Error Matrix:\n{error_matrix}")
        rospy.loginfo(f"{description} Position Error: {position_error}")
        rospy.loginfo(f"{description} Orientation Error (Rotation Matrix):\n{orientation_error}")

    def main(self):
        rospy.loginfo("Waiting for joint states...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.joint_angles is None:
            rate.sleep()

        if self.joint_angles is not None:
            self.home_position, home_orientation = self.forward_kinematics(self.joint_angles)
            self.log_position_orientation(self.home_position, home_orientation, "Starting Position")
            self.log_joint_info(self.joint_angles, "Starting Position")

        pick_joint_angles = self.inverse_kinematics(self.pick_position, self.joint_angles)
        if pick_joint_angles is not None:
            self.send_joint_trajectory(pick_joint_angles, "Pick")
            rospy.sleep(5)
            pick_position, pick_orientation = self.forward_kinematics(pick_joint_angles)
            self.log_position_orientation(pick_position, pick_orientation, "Pick Position")
            self.log_joint_info(pick_joint_angles, "Pick Position")
            self.compute_error_matrix(self.pick_position, pick_position, np.eye(3), pick_orientation, "Pick")

        place_joint_angles = self.inverse_kinematics(self.place_position, pick_joint_angles)
        if place_joint_angles is not None:
            self.send_joint_trajectory(place_joint_angles, "Place")
            rospy.sleep(5)
            place_position, place_orientation = self.forward_kinematics(place_joint_angles)
            self.log_position_orientation(place_position, place_orientation, "Place Position")
            self.log_joint_info(place_joint_angles, "Place Position")
            self.compute_error_matrix(self.place_position, place_position, np.eye(3), place_orientation, "Place")

        home_joint_angles = self.inverse_kinematics(self.home_position, place_joint_angles)
        if home_joint_angles is not None:
            self.send_joint_trajectory(home_joint_angles, "Home")
            rospy.sleep(5)
            home_position, home_orientation = self.forward_kinematics(home_joint_angles)
            self.log_position_orientation(home_position, home_orientation, "Home Position")
            self.log_joint_info(home_joint_angles, "Home Position")
            self.compute_error_matrix(self.home_position, home_position, np.eye(3), home_orientation, "Home")

        rospy.loginfo("Pick-and-place routine completed.")


if __name__ == "__main__":
    pnp = PickAndPlace()
    pnp.main()
