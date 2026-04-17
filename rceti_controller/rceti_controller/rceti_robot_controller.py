# MODIFICATION NOTICE
# This file is part of a derivative work based on the original RCETI project (https://github.com/bturner86239/RCETI).
# It was modified by CSE 2.3 in March, 2026 in accordance with Section 4(b) of the Apache License 2.0.

# Major Changes:
# Added support for two new continuum_motors.
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time  # for delays/testing

# A try catch block to detect the raspberry pi. Enters mock mode for simulation testing if it is not detected.
try:
    import lgpio
    from adafruit_servokit import ServoKit
    import adafruit_motor.servo
    HARDWARE_MODE = True
    kit = ServoKit(channels=16)
    print("Raspberry Pi Hardware detected. Running in HARDWARE mode.")

except (ImportError, NotImplementedError):
    print("WARNING: Raspberry Pi Hardware not detected. Running in MOCK SIMULATION mode.")
    HARDWARE_MODE = False

    class MockServo:
        """A mock representation of an Adafruit servo motor for simulation testing."""
        
        def __init__(self):
            """Initializes the mock servo with a default actuation range."""
            self._angle = None
            self.actuation_range = 180
            
        def set_pulse_width_range(self, min_p, max_p):
            """Mocks the hardware method for setting the pulse width range.
            
            Args:
                min_p (int): The minimum pulse width in microseconds.
                max_p (int): The maximum pulse width in microseconds.
            """
            pass

        @property
        def angle(self):
            """Gets the current angle of the simulated servo.
            
            Returns:
                float: The current angle.
            """
            return self._angle
        
        @angle.setter
        def angle(self, val): 
            """Sets the angle of the simulated servo.
            
            Args:
                val (float): The desired angle to set.
            """
            self._angle = val

    class MockServoKit:
        """A mock representation of the Adafruit servo controller."""

        def __init__(self, channels):
            """Initializes the mock servo kit with a specified number of channels.
            
            Args:
                channels (int): The total number of servo channels available on the board.
            """
            self.servo = [MockServo() for _ in range(channels)]

    class MockLgpio:
        """A mock representation of the lgpio library for simulating Raspberry Pi GPIO pins."""

        def gpiochip_open(self, chip): 
            """Mocks opening the GPIO chip for access.
            
            Args:
                chip (int): The GPIO chip number to open.
                
            Returns:
                int: A dummy handle ID for the opened chip.
            """
            return 1
        
        def gpio_claim_output(self, chip, pin): 
            """Mocks claiming a specific GPIO pin for digital output.
            
            Args:
                chip (int): The handle ID of the GPIO chip.
                pin (int): The physical pin number to claim.
            """
            pass

        def gpio_write(self, chip, pin, val):
            """Mocks writing a digital high or low state to a GPIO pin.
            
            Args:
                chip (int): The handle ID of the GPIO chip.
                pin (int): The physical pin number to write to.
                val (int): The value to write (1 for HIGH, 0 for LOW).
            """
            pass

    kit = MockServoKit(channels=16)
    lgpio = MockLgpio()

TOPIC_SUBSCRIPTION_BUFFER = 5

class RCETIRobotController(Node):
    """RCETIRobotController is a ROS 2 node that controls the RCETI robot's stepper motors and pitch servo.

    Args:
        Node (Node): The node of the ROS 2 system that handles the robot's control logic.
    """

    def __init__(self):
        """Initializes the RCETIRobotController node, subscribes to the /joint_states topic, and sets up GPIO pins for stepper motors and servo motors."""
        
        super().__init__('rceti_controller')

        # Initialize servo motors
        self.servo1 = kit.servo[0]       # tilts continuum base
        self.servo2 = kit.servo[4]       # pulls continuum (N)
        self.servo3 = kit.servo[2]       # pulls continuum (S)
        self.servo4 = kit.servo[3]       # pulls continuum (W)
        self.servo5 = kit.servo[1]       # pulls continuum (E)

        self.servo1.actuation_range = 200
        self.servo1.set_pulse_width_range(500, 2000)

        # CONTINUUM SERVO BOUND CONSTANTS
        self.continuum_servo_half_range = 20
        self.continuum_servo_middle = 90

        # May need to adjust 
        self.servo2.actuation_range = 180
        self.servo2.set_pulse_width_range(500, 2500)
        # May need to adjust 
        self.servo3.actuation_range = 180
        self.servo3.set_pulse_width_range(500, 2500)
        # May need to adjust 
        self.servo4.actuation_range = 180
        self.servo4.set_pulse_width_range(500, 2500)
        # May need to adjust 
        self.servo5.actuation_range = 180
        self.servo5.set_pulse_width_range(500, 2500)

        # Subscribe to the /joint_states topic
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            TOPIC_SUBSCRIPTION_BUFFER
        )

        # Position tracking
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.730

        # GPIO pin definitions
        self.X_DIRECTION_PIN = 21
        self.X_PULSE_PIN = 20
        self.Z_DIRECTION_PIN = 19
        self.Z_PULSE_PIN = 18

        # Open the GPIO chip and set the pins as output
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.X_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.X_PULSE_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_PULSE_PIN)

        # Define stepper steps per unit in X and Z directions
        self.steps_per_mm_x = 40
        self.steps_per_mm_z = 40

    def joint_state_callback(self, msg):
        """Handles incoming joint state messages and moves the stepper motors accordingly.
        This function is called whenever a new message is received on the /joint_states topic.
        Moves the x stepper motor, z stepper motor, and pitch servo motor based on the joint states received.

        Args:
            msg (sensor_msgs/JointState.msg): The messages recieved from the /joint_states topic.
        """
        # Extract joint positions from the message
        try:
            x_index = msg.name.index('x_actuator_to_x_slider')
            z_index = msg.name.index('z_actuator_to_z_slider')
            pitch_index = msg.name.index('z_slider_to_pitch_servo')
            continuum_index_1 = msg.name.index('continuum_motor_1')
            continuum_index_2 = msg.name.index('continuum_motor_2')
            continuum_index_3 = msg.name.index('continuum_motor_3')
            continuum_index_4 = msg.name.index('continuum_motor_4')

            new_x_position = msg.position[x_index]
            new_z_position = msg.position[z_index]
            pitch_angle_msg = msg.position[pitch_index]
            continuum_angle_1 = msg.position[continuum_index_1]
            continuum_angle_2 = msg.position[continuum_index_2]
            continuum_angle_3 = msg.position[continuum_index_3]
            continuum_angle_4 = msg.position[continuum_index_4]

            # Handle X-axis movement
            if self.x_position != new_x_position:
                self.get_logger().info(f"Moving X to {new_x_position}")
                steps = int(abs(new_x_position - self.x_position) * 1000 * self.steps_per_mm_x)
                direction = 1 if new_x_position > self.x_position else 0
                self.move_stepper(steps, direction, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
                self.x_position = new_x_position

            # Handle Z-axis movement
            if self.z_position != new_z_position:
                self.get_logger().info(f"Moving Z to {new_z_position}")
                steps = int(abs(new_z_position - self.z_position) * 1000 * self.steps_per_mm_z)
                direction = 1 if new_z_position > self.z_position else 0
                self.move_stepper(steps, direction, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
                self.z_position = new_z_position

            # Handle pitch and continuum angles with safety clamps
            new_pitch_angle = self.clamp_angle(((pitch_angle_msg + 0.475) / 1.205) * 120)
            
            # new_continuum_1_pitch_angle = self.clamp_angle(((continuum_angle_1 + 0.475) / 1.205) * 120)
            # new_continuum_2_pitch_angle = self.clamp_angle(((continuum_angle_2 + 0.475) / 1.205) * 120)
            # new_continuum_3_pitch_angle = self.clamp_angle(((continuum_angle_3 + 0.475) / 1.205) * 120)
            # new_continuum_4_pitch_angle = self.clamp_angle(((continuum_angle_4 + 0.475) / 1.205) * 120)

            servo1_angle = self.clamp_continuum_angle(self.convert_continuum_angle(continuum_angle_1))
            servo2_angle = self.clamp_continuum_angle(self.convert_continuum_angle(continuum_angle_2))
            servo3_angle = self.clamp_continuum_angle(self.convert_continuum_angle(continuum_angle_3))
            servo4_angle = self.clamp_continuum_angle(self.convert_continuum_angle(continuum_angle_4))

            if (self.servo1.angle != new_pitch_angle): 
                self.get_logger().info(f"Adjusting pitch to {new_pitch_angle}")
                self.servo1.angle = new_pitch_angle

            if (self.servo2.angle != servo1_angle): 
                self.get_logger().info(f"Adjusting continuum 1 to {servo1_angle}")
                self.servo2.angle = servo1_angle

            if (self.servo3.angle != servo2_angle): 
                self.get_logger().info(f"Adjusting continuum 2 to {servo2_angle}")
                self.servo3.angle = servo2_angle
                
            if (self.servo4.angle != servo3_angle): 
                self.get_logger().info(f"Adjusting continuum 3 to {servo3_angle}")
                self.servo4.angle = servo3_angle

            if (self.servo5.angle != servo4_angle): 
                self.get_logger().info(f"Adjusting continuum 4 to {servo4_angle}")
                self.servo5.angle = servo4_angle

        except ValueError as e:
            self.get_logger().error(f"Joint name not found in joint_states: {e}")

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.001):
        """Stepper Motor Helper Function, handles all stepper movement

        Args:
            steps (int): the amount of "steps" the stepper motor will take
            direction (int/bool): the direction the stepper motor will move, 0 is for backward, 1 is for forward
            direction_pin (int): the GPIO pin used to set the direction of the stepper motor
            pulse_pin (int): the GPIO pin used to pulse the stepper motor
            delay (float, optional): the delay between steps, defaults to 0.001.
        """
        
        if HARDWARE_MODE:
            self.get_logger().info(f"Hardware pulsing: steps={steps}, dir={direction}")
            lgpio.gpio_write(self.chip, direction_pin, direction)

            for _ in range(steps):
                lgpio.gpio_write(self.chip, pulse_pin, 1)
                time.sleep(delay)
                lgpio.gpio_write(self.chip, pulse_pin, 0)
                time.sleep(delay)
        else:
            # Skip time.sleep() so it is reflected in simulation instanly
            self.get_logger().info(f"[MOCK] Simulated moving stepper: steps={steps}, dir={direction}")

    def clamp_angle(self, value):
        """Clamps the calculated angle between 0 and 120 degrees."""
        return max(0, min(120, int(value)))

    def convert_continuum_angle(self, value):
        """
        Converts value from joint state (-0.5, 0.5) to servo angle
        (continuum_servo_middle +- continuum_servo_half_angle).
        """
        return (value * self.continuum_servo_half_range * 2) + 90

    def clamp_continuum_angle(self, value):
        """Clamps the angle between the min and max range set by constants."""
        return max(
            self.continuum_servo_middle - self.continuum_servo_half_range,
            min(
                self.continuum_servo_middle + self.continuum_servo_half_range,
                value
            )
        )

def main(args=None):
    """The main function initializes the ROS 2 node and starts the RCETIRobotController.

    Args:
        args (N/A optional):Defaults to None, shouldn't be set to anything
    """
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()