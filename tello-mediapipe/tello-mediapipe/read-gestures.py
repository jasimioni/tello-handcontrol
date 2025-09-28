import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import math
import numpy as np
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from tello_msgs.msg import FlightData

import time

CALIBRATION_CONSTANT = 5000 
image_width = 640  # Default width, adjust as needed
image_height = 480 # Default height, adjust as needed

def is_hand_open(hand_landmarks):
    """
    Checks if a hand is open by comparing the y-coordinates of the fingertips
    to the y-coordinates of the knuckles below them.
    """
    # Landmark indices for the tips of the four fingers (excluding thumb)
    finger_tip_indices = [8, 12, 16, 20]
    # Landmark indices for the PIP joint (one joint below the tip)
    finger_pip_indices = [6, 10, 14, 18]
    
    extended_fingers = 0
    for i in range(4):
        tip = hand_landmarks.landmark[finger_tip_indices[i]]
        pip = hand_landmarks.landmark[finger_pip_indices[i]]
        
        # In the landmark coordinates, a lower y-value means it's higher up on the image.
        if tip.y < pip.y:
            extended_fingers += 1
            
    return "Open" if extended_fingers >= 4 else "Closed"

def calculate_palm_normal(hand_landmarks):
    """
    Calculates the normal vector of the palm.
    
    Returns:
        A normalized 3D numpy array representing the normal vector, or None if calculation fails.
    """
    try:
        # Get the 3D coordinates of the three palm landmarks
        p0 = hand_landmarks.landmark[0]  # Wrist
        p5 = hand_landmarks.landmark[5]  # Index finger MCP
        p17 = hand_landmarks.landmark[17] # Pinky MCP

        # Convert them to numpy arrays
        p0 = np.array([p0.x, p0.y, p0.z])
        p5 = np.array([p5.x, p5.y, p5.z])
        p17 = np.array([p17.x, p17.y, p17.z])

        # Create two vectors on the palm plane
        v1 = p5 - p0
        v2 = p17 - p0

        # Calculate the cross product to get the normal vector
        normal = np.cross(v1, v2)
        
        # Normalize the vector to make it a unit vector (length of 1)
        norm_normal = normal / np.linalg.norm(normal)
        
        norm_normal = [ f'{coord:.2f}' for coord in norm_normal ]
        
        return norm_normal
        
    except Exception as e:
        # Could fail if landmarks are not detected or norm is zero
        return None

def get_hand_centered_status(hand_landmarks, tolerance=0.1):
    """
    Checks if the hand is centered horizontally and vertically.
    """
    wrist = hand_landmarks.landmark[0]
    
    h_status = f'{wrist.x:.1f}'
    v_status = f'{wrist.y:.1f}'
    
    return h_status, v_status


# --- YOU MUST CALIBRATE THIS VALUE FOR YOUR CAMERA AND HAND SIZE ---
# F = (P * D) / W, but we simplify by using a pixel distance P directly.
# My calibration constant was roughly 5000. You will need to find your own.

def estimate_distance(hand_landmarks, image_width, image_height):
    """
    Estimates the distance of the hand from the camera based on its apparent size.
    """
    # Get pixel coordinates of wrist and middle finger MCP joint
    wrist_px = (hand_landmarks.landmark[0].x * image_width, 
                hand_landmarks.landmark[0].y * image_height)
    mcp_px = (hand_landmarks.landmark[9].x * image_width, 
              hand_landmarks.landmark[9].y * image_height)

    # Calculate the pixel distance between these two points
    pixel_dist = math.sqrt((wrist_px[0] - mcp_px[0])**2 + (wrist_px[1] - mcp_px[1])**2)
    
    if pixel_dist == 0:
        return -1 # Avoid division by zero
        
    # Estimate distance using the calibrated constant
    distance_cm = CALIBRATION_CONSTANT / pixel_dist
    
    return distance_cm


class MediaPipeHandsNode(Node):
    """
    A ROS 2 node that subscribes to an image topic, performs hand detection
    using MediaPipe, and republishes the image with landmarks.
    """
    def __init__(self):
        super().__init__('mediapipe_hands_node')
        
        # 3. Initialize MediaPipe Hands
        # It's important to initialize this once to avoid reloading the model
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.flying = False
        self.takeoff_count = 0
        self.future = None
        self.pending_action = None
        self.last_flight_data = None
        self.landed = True
        self.landing = False
        self.land_check_count = 0
        self.flying_check_count = 0
        
        self.multi_hand_detected_count = 0
        self.no_hands_detected_count = 0
        # 1. Initialize the CvBridge
        self.bridge = CvBridge()
        
        # 2. Create subscriber and publisher
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/image_processed', 10)
    
        self.cli = self.create_client(TelloAction, '/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()
        
        #self.tello_action('takeoff')
        #time.sleep(10)
        #self.tello_action('land')
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
    
        # Create a timer that calls self.timer_callback every 0.1 seconds
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.flight_data_sub = self.create_subscription(
            FlightData,
            '/flight_data',
            self.flight_data_callback,
            10
        )


        self.get_logger().info('MediaPipe Hands node has been started.')

    def flight_data_callback(self, msg):
        # Process flight data message here
        # self.get_logger().info(f'Received flight data message. {msg.tof} cm, {msg.bat}%')
        
        self.last_flight_data = msg

    def timer_callback(self):
        """
        Timer callback for periodic tasks.
        """

        if self.pending_action is not None:
            self.get_logger().info(f'Pending action: {self.pending_action} - {self.future}')
            if self.future is not None and self.future.done():
                try:
                    response = self.future.result()
                    self.get_logger().info(f'Service call succeeded with response: {response}')
                    if response.rc == 1:
                        if self.pending_action == 'takeoff':
                            self.landed = False
                        elif self.pending_action == 'land':
                            self.flying = False
                            self.landing = True
                
                except Exception as e:
                    self.get_logger().error(f'Service call failed with exception: {e}')

                self.pending_action = None
                self.future = None
            else:
                self.get_logger().info(f'Waiting for service call {self.pending_action} to complete...')
                
        if not self.flying and not self.landed:
            if self.last_flight_data is not None:
                if self.last_flight_data.tof < 30:
                    self.land_check_count += 1
                    if self.land_check_count > 25:
                        self.get_logger().info(f'Close to ground, marking as landed. Battery: {self.last_flight_data.bat}')
                        self.landed = True
                        self.flying = False
                        self.landing = False
                        self.land_check_count = 0
                elif not self.landing:
                    self.flying_check_count += 1
                    if self.flying_check_count > 10:
                        self.get_logger().info(f'Away from the ground, marking as flying. Battery: {self.last_flight_data.bat}')
                        self.landed = False
                        self.flying = True
                        self.flying_check_count = 0

    def tello_action(self, action):
        """
        Publish a Tello action command.
        """
        
        if self.pending_action is not None:
            self.get_logger().info(f'Attempting to perform action: {self.pending_action}')
            return

        self.pending_action = action
        
        self.req.cmd = action
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Published Tello action: {action}')

    def adjust_flight(self, hand_state, h_status, v_status, normal_vector, distance):
        # hand_state = "Open" or "Closed"
        # h_status from 0.0 to 1.0
        # v_status from 0.0 to 1.0
        # normal_vector = [x, y, z] each from -1.0 to 1.0
        # distance = estimated distance in cm
        
        if not self.flying:
            return
        
        cmd_vel = Twist()
        
        self.get_logger().info(f'Hand: {hand_state}, H: {h_status}, V: {v_status}, Normal: {normal_vector}, Dist: {distance:.1f} cm')
        
        if hand_state == "Closed":
            expected_distance = 50.0  # Closer when hand is closed
        else:
            expected_distance = 100.0  # Further when hand is open
            
        distance = float(distance)
        h_status = float(h_status)
        v_status = float(v_status)
            
        
        if abs(distance - expected_distance) > 10:
            if float(distance) > float(expected_distance):
                self.get_logger().info('Too far, moving forward')
                cmd_vel.linear.x = 0.2
            elif float(distance) < float(expected_distance):
                self.get_logger().info('Too close, moving backward')
                cmd_vel.linear.x = -0.2
        else:
            self.get_logger().info("At the right spot")
            cmd_vel.linear.x = 0.0
            
        if h_status < 0.4:
            self.get_logger().info('Hand left, moving left')
            cmd_vel.linear.y = 0.2
            cmd_vel.angular.z = 0.2
        elif float(h_status) > 0.6:
            self.get_logger().info('Hand right, moving right')
            cmd_vel.linear.y = -0.2
            cmd_vel.angular.z = -0.2
        else:
            self.get_logger().info('Hand is horizontally ok - holding')
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            
        if float(v_status) < 0.4:
            self.get_logger().info('Hand up, moving up')
            cmd_vel.linear.z = 0.2
        elif float(v_status) > 0.6:
            self.get_logger().info('Hand down, moving down')
            cmd_vel.linear.z = -0.2
        else:
            self.get_logger().info('Hand height is good')
            cmd_vel.linear.z = 0.0
            
        #if normal_vector is not None:
        #    if float(normal_vector[0]) > 0.1:
        #        self.get_logger().info('Palm tilted right, rotating right')
        #        cmd_vel.angular.z = 0.3
        #    elif float(normal_vector[0]) < -0.1:
        #        self.get_logger().info('Palm tilted left, rotating left')
        #        cmd_vel.angular.z = -0.3
        #    else:
        #        self.get_logger().info('Palm is good. Holding.')
        #        cmd_vel.angular.z = 0.0
            
        self.get_logger().info(f"Sending: {cmd_vel}")
        self.cmd_vel_pub.publish(cmd_vel)
        
    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        """
        try:
            # Convert the ROS 2 Image message to an OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # --- MediaPipe Processing ---
        # For performance, mark the image as not writeable
        # cv_image.flags.setflags(write=0)
        
        # MediaPipe expects RGB, but OpenCV provides BGR. Convert the image.
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Process the image and find hands
        results = self.hands.process(rgb_image)
        
        # Re-enable writing to the image to draw on it
        # cv_image.flags.setflags(write=1)
        
        # Draw the hand annotations on the image
        if results.multi_hand_landmarks:
            
            num_hands = len(results.multi_hand_landmarks)
            
            if self.landed and num_hands >= 1:
                self.takeoff_count += 1
                if self.takeoff_count > 20:
                    self.takeoff_count = 0
                    self.tello_action('takeoff')
            
            #if self.flying:
            #    if num_hands > 1:
            #        self.no_hands_detected_count = 0
            #        self.multi_hand_detected_count += 1
            #        if self.multi_hand_detected_count > 200:
            #            self.get_logger().info('Multiple hands detected for a while, landing...')
            #            self.multi_hand_detected_count = 0
            #            self.tello_action('land')
            self.get_logger().info(f'Detected {num_hands} hand(s).')
            
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    cv_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS)

            # --- Perform our calculations ---
            hand_state = is_hand_open(hand_landmarks)
            h_status, v_status = get_hand_centered_status(hand_landmarks)
            normal_vector = calculate_palm_normal(hand_landmarks)
            distance = estimate_distance(hand_landmarks, image_width, image_height)

            # hand_state = "Open" or "Closed"
            # h_status from 0.0 to 1.0
            # v_status from 0.0 to 1.0
            # normal_vector = [x, y, z] each from -1.0 to 1.0
            # distance = estimated distance in cm
            
            self.adjust_flight(hand_state, h_status, v_status, normal_vector, distance)
            
            
            
            # --- Display the results on the image ---
            # Position text near the wrist (landmark 0)
            wrist_coords = (int(hand_landmarks.landmark[0].x * image_width), 
                            int(hand_landmarks.landmark[0].y * image_height))
            
            info_text = f"{hand_state}, {h_status} {v_status}"
            dist_text = f"Dist: {distance:.1f} cm / {normal_vector}"

            cv2.putText(cv_image, info_text, (wrist_coords[0] + 10, wrist_coords[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, dist_text, (wrist_coords[0] + 10, wrist_coords[1] + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
        else:
            # No hands detected... let's land if needed
            if self.flying:
                self.no_hands_detected_count += 1
                if self.no_hands_detected_count and not self.no_hands_detected_count % 50:
                    self.get_logger().info('No hands detected, hovering...')
                    hover_cmd = Twist()
                    self.cmd_vel_pub.publish(hover_cmd)
                if self.no_hands_detected_count > 250:
                    self.no_hands_detected_count = 0
                    self.get_logger().info('No hands detected for a while, landing...')
                    self.tello_action('land')
            


        # --- Publishing the result ---
        try:
            # Convert the OpenCV image with drawings back to a ROS 2 Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            # The header of the output message should match the input message
            processed_image_msg.header = msg.header
            self.publisher.publish(processed_image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert and publish image: {e}')


def main(args=None):
    rclpy.init(args=args)
    mediapipe_node = MediaPipeHandsNode()
    try:
        rclpy.spin(mediapipe_node)
    except KeyboardInterrupt:
        pass
    finally:
        mediapipe_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()