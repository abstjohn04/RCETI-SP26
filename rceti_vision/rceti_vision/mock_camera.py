import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MockCamera(Node):
    """
    @class MockCamera
    @brief Simulates a live camera feed using a local video file.
    
    This node creates a publisher on the '/camera/image_raw' topic and uses 
    a timer to read and broadcast frames from a surgical video. 
    If the video reaches the end, it automatically loops back to the beginning.
    """
    def __init__(self):
        """
        @brief Initializes the MockCamera node.
        
        Sets up the ROS publisher, initializes the OpenCV video capture object, 
        and creates the 30Hz timer loop.
        """
        super().__init__('mock_camera')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # CHANGE ME
        video_path = '/home/cse5911/ML/full_videos_for_prediction/000090258_001.mp4'
        self.cap = cv2.VideoCapture(video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video file at {video_path}")
            return

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Mock Camera started. Broadcasting video...")

    def timer_callback(self):
        """
        @brief Callback function triggered every 1/30th of a second.
        
        Reads the next frame from the video file, converts it to a ROS 2 
        Image message, and publishes it. If the end of the video is reached, 
        the frame pointer is reset to frame 0 to loop the video indefinitely.
        """
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().info("Video finished playing. Looping...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    """
    @brief Main entry point for the mock_camera node.
    @param args Command line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    node = MockCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()