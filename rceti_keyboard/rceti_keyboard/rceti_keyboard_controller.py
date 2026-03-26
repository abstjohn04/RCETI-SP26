import sys
import termios
import tty
import time
from select import select
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

LINEAR_STEP = 0.01
FIVE_DEGREES_FUNCTIONAL = 0.050208333
class Press:
    UP_ARROW = '\x1b[A'
    DOWN_ARROW = '\x1b[B'
    RIGHT_ARROW = '\x1b[C'
    LEFT_ARROW = '\x1b[D'
    CTRL_C = '\x03'
    ESC = '\x1b'
class RcetiKeyboardController(Node):
    """RcetiKeyboardController is a ROS 2 node that handles keyboard input to control the RCETI robot's joint states.

    Args:
        Node (Node): The node of the ROS 2 system that handles the keyboard input and joint state publishing.
    """

    def __init__(self):
        """Initializes the RcetiKeyboardController node, sets up the publisher for joint states, and initializes parameters.
        """
        super().__init__('rceti_keyboard')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.MIN_X_POSITION, self.MAX_X_POSITION = 0.0, 0.309
        self.MIN_Z_POSITION, self.MAX_Z_POSITION = 0.0, 0.309
        self.MIN_PITCH_ANGLE, self.MAX_PITCH_ANGLE  = -0.475, 0.730
        self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE = -0.5, 0.5
        
        self.x_position = self.MIN_X_POSITION
        self.z_position = self.MIN_Z_POSITION
        self.pitch_angle =  self.MAX_PITCH_ANGLE
        self.continuum_motor_1_angle = self.continuum_motor_2_angle = self.continuum_motor_3_angle = self.continuum_motor_4_angle = 0.0

        self.original_settings = termios.tcgetattr(sys.stdin)

        self.declare_parameter('key_timeout', 0.1)

        self.keyboard_timer = self.create_timer(0.01, self.keyboard_callback)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
    def _detectKey(self, timeout):
        """Detects a key press from the keyboard with a timeout.

        Args:
            timeout (float): The timeout duration in seconds.

        Returns:
            string: The key pressed or an empty string if no key was pressed within the timeout.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)

        if rlist:
            keysPressed = sys.stdin.read(1)  # Read the first character
            if keysPressed == Press.ESC:  # Escape character
                keysPressed += sys.stdin.read(2)  # Read the '[' and the final character (e.g., 'A', 'B', 'C', or 'D')
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            time.sleep(0.05)
            return keysPressed
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
        return ''

    def _update_joint(self, new_val, min_val, max_val, name):
        """Helper method to cleanly update a joint value and enforce hardware boundaries.
        """
        if new_val > max_val:
            self.get_logger().info(f"Cannot increase {name}, at maximum limit.")
            return max_val
        elif new_val < min_val:
            self.get_logger().info(f"Cannot decrease {name}, at minimum limit.")
            return min_val
        return new_val
    
    def keyboard_callback(self):
        """Handles keyboard input to control the robot's joint states."""
        
        keyPressed = self._detectKey(self.get_parameter("key_timeout").get_parameter_value().double_value)

        if not keyPressed: return
        
        match keyPressed:
            case 'd': self.x_position = self._update_joint(self.x_position - LINEAR_STEP, self.MIN_X_POSITION, self.MAX_X_POSITION, "X Position")
            case 'a': self.x_position = self._update_joint(self.x_position + LINEAR_STEP, self.MIN_X_POSITION, self.MAX_X_POSITION, "X Position")
            case 'w': self.z_position = self._update_joint(self.z_position - LINEAR_STEP, self.MIN_Z_POSITION, self.MAX_Z_POSITION, "Z Position")
            case 's': self.z_position = self._update_joint(self.z_position + LINEAR_STEP, self.MIN_Z_POSITION, self.MAX_Z_POSITION, "Z Position")

            case 'p': self.pitch_angle = self._update_joint(self.pitch_angle - FIVE_DEGREES_FUNCTIONAL, self.MIN_PITCH_ANGLE, self.MAX_PITCH_ANGLE, "Pitch Angle")
            case 'l': self.pitch_angle = self._update_joint(self.pitch_angle + FIVE_DEGREES_FUNCTIONAL, self.MIN_PITCH_ANGLE, self.MAX_PITCH_ANGLE, "Pitch Angle")
            
            case Press.UP_ARROW: 
                self.continuum_motor_1_angle = self._update_joint(self.continuum_motor_1_angle + FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 1")
                self.continuum_motor_2_angle = self._update_joint(self.continuum_motor_2_angle - FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 2")
            case Press.DOWN_ARROW: 
                self.continuum_motor_1_angle = self._update_joint(self.continuum_motor_1_angle - FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 1")
                self.continuum_motor_2_angle = self._update_joint(self.continuum_motor_2_angle + FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 2")

            case Press.RIGHT_ARROW: 
                self.continuum_motor_3_angle = self._update_joint(self.continuum_motor_3_angle + FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 3")
                self.continuum_motor_4_angle = self._update_joint(self.continuum_motor_4_angle - FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 4")
            case Press.LEFT_ARROW: 
                self.continuum_motor_3_angle = self._update_joint(self.continuum_motor_3_angle - FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 3")
                self.continuum_motor_4_angle = self._update_joint(self.continuum_motor_4_angle + FIVE_DEGREES_FUNCTIONAL, self.MIN_CONTINUUM_ANGLE, self.MAX_CONTINUUM_ANGLE, "Continuum 4")

            case Press.CTRL_C:
                rclpy.shutdown()
                sys.exit(0)

            case _: pass

    def publish_joint_states(self):
        """Publishes the current joint states to the /joint_states topic.
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['x_actuator_to_x_slider', 'z_actuator_to_z_slider', 'z_slider_to_pitch_servo', 'continuum_motor_1', 'continuum_motor_2', 'continuum_motor_3', 'continuum_motor_4']
        joint_state.position = [self.x_position, self.z_position, self.pitch_angle, self.continuum_motor_1_angle, self.continuum_motor_2_angle, self.continuum_motor_3_angle, self.continuum_motor_4_angle]
        self.joint_state_publisher.publish(joint_state)

        self.get_logger().info(f"Published Joint States: X: {self.x_position:.3f}, Z: {self.z_position:.3f}, Pitch: {self.pitch_angle:.3f}, Continuum Motor 1: {self.continuum_motor_1_angle:.3f}, Continuum Motor 2: {self.continuum_motor_2_angle:.3f},  Continuum Motor 3:{self.continuum_motor_3_angle:.3f}  Continuum Motor 4:{self.continuum_motor_4_angle:.3f}")


def main(args=None):
    """Main function to initialize the RcetiKeyboardController node and start the ROS 2 event loop.

    Args:
        args (N/A optional):Defaults to None, shouldn't be set to anything
    """
    rclpy.init(args=args)
    keyboard_controller = RcetiKeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()