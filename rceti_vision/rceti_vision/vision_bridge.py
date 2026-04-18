import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class VisionBridge(Node):
    """
    ROS 2 Node that bridges the YOLOv8 Machine Learning pipeline with the RCETI kinematics.
    Subscribes to camera feeds, runs inference, and publishes target coordinates.
    """
    def __init__(self):
        super().__init__('vision_bridge')

        self.bridge = CvBridge()

        try:
            model_path = os.path.join(get_package_share_directory('rceti_vision'), 'models', 'best.pt')
            
            self.get_logger().info(f"Loading YOLO weights from: {model_path}")
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO Model loaded successfully!")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model. Error: {e}")
            raise e

        self.image_sub = self.create_subscription( Image, '/camera/image_raw', self.image_callback, 10 )

        self.kinematics_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.declare_parameter('yaw_step_size', 0.00001)
        self.declare_parameter('pitch_step_size', 0.0001)

        self.yaw_step_size = self.get_parameter('yaw_step_size').value
        self.pitch_step_size = self.get_parameter('pitch_step_size').value

        self.current_yaw = 0.0
        self.current_pitch = 0.0

        self.get_logger().info("Vision Bridge Initialized.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            height, width, _ = cv_image.shape

            results = self.model.predict(cv_image, verbose=False)

            for result in results:
                for box in result.boxes:             
                    if int(box.cls[0]) == 0 and float(box.conf[0]) > 0.6:
                        target_x, target_y, _, _ = box.xywh[0].tolist()

                        self.current_yaw += ((target_x -  width / 2.0) * self.yaw_step_size)
                        self.current_pitch += ((target_y - height / 2.0) * self.pitch_step_size)
                        
                        self.get_logger().info(
                            f"Steering | Yaw: {self.current_yaw:.3f} rad, Pitch: {self.current_pitch:.3f} rad", 
                            throttle_duration_sec=0.5
                        )
                        break 
            self.publish_kinematics()

        except Exception as e:
            self.get_logger().error(f"Vision Bridge Error: {e}")

    def publish_kinematics(self):
        """Heartbeat publisher: Constantly broadcasts the robot skeleton to keep RViz alive."""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        joint_msg.name = [
            'x_actuator_to_x_slider', 
            'z_actuator_to_z_slider', 
            'z_slider_to_pitch_servo',
            'continuum_motor_1',
            'continuum_motor_2', 
            'continuum_motor_3',
            'continuum_motor_4'  
        ]
        
        joint_msg.position = [
            0.0,                 # x_slider (Anchored)
            0.0,                 # z_slider (Anchored)
            0.730,               # pitch_servo (Anchored)
            self.current_pitch,  # AI Memory
            0.0,                 # Ghost
            self.current_yaw,    # AI Memory
            0.0                  # Ghost
        ]
        
        self.kinematics_pub.publish(joint_msg)

        self.get_logger().info("AI Heartbeat: Pulse sent to RViz.", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisionBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()