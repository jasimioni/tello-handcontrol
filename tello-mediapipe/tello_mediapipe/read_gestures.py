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

class PIDController:
    """A simple PID controller class."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        """Calculates the PID output."""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def clean(self):
        self.prev_error = 0.0
        self.integral = 0.0

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
        
        self.MAX_LINEAR_SPEED = 1.0  # m/s
        self.MAX_ANGULAR_SPEED = 1.0 # rad/s
        
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
        self.last_time = None
        self.last_expected_distance = None

        self.pid_forward = PIDController(kp=0.4, ki=0.001, kd=0.005)   # Controls linear.x
        self.pid_vertical = PIDController(kp=1.2, ki=0.2, kd=0.3)      # Controls linear.z
        self.pid_sideways = PIDController(kp=0.5, ki=0.1, kd=0.1)      # Controls linear.y
        self.pid_yaw = PIDController(kp=0.8, ki=0.1, kd=0.2)           # Controls angular.z
        
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
        if not self.flying:
            return
    
        # hand_state = "Open" or "Closed"
        # h_status from 0.0 to 1.0
        # v_status from 0.0 to 1.0
        # normal_vector = [x, y, z] each from -1.0 to 1.0
        # distance = estimated distance in cm
        
        distance = float(distance)
        h_status = float(h_status)
        v_status = float(v_status)

        # Calculate time delta (dt) for PID controllers
        current_time = time.time()
        if not self.last_time:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt == 0:
            return

        # --- 1. Define Setpoints (Target Values) ---
        # Setpoint for distance
        expected_distance = 80.0 if hand_state == "Closed" else 130.0
        
        # Reset PID integrals if the goal changes to prevent sudden jumps
        if expected_distance != self.last_expected_distance:
            self.get_logger().info("Hand state changed, resetting PID controllers.")
            self.pid_forward.clean()
            self.last_expected_distance = expected_distance

        # Setpoint for horizontal and vertical position is the center of the screen (0.5)
        h_setpoint = 0.5
        v_setpoint = 0.5

        # If hand is to the right (h_status > 0.5), error is negative -> move right (negative y)
        h_error = h_setpoint - h_status
        
        # If hand is down (v_status > 0.5), error is negative -> move down (negative z)
        v_error = v_setpoint - v_status
        
        # Let's redefine the error to be more intuitive
        # Positive error = move forward. Negative error = move back
        distance_error = (distance - expected_distance) / 100.0

        forward_speed = self.pid_forward.update(distance_error, dt)
        vertical_speed = self.pid_vertical.update(v_error, dt)
        sideways_speed = self.pid_sideways.update(h_error, dt)
        yaw_speed = self.pid_yaw.update(h_error, dt)

        # --- 4. Clamp Outputs to Safe Maximums ---
        forward_speed = np.clip(forward_speed, -self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED)
        vertical_speed = np.clip(vertical_speed, -self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED)
        sideways_speed = np.clip(sideways_speed, -self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED)
        yaw_speed = np.clip(yaw_speed, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
        
        # --- 5. Publish Command ---
        cmd_vel = Twist()
        cmd_vel.linear.x = forward_speed
        cmd_vel.linear.y = sideways_speed
        cmd_vel.linear.z = vertical_speed
        cmd_vel.angular.z = yaw_speed
        
        self.get_logger().info(f"Errors(d,h,v):({distance_error:.2f}, {h_error:.2f}, {v_error:.2f}) -> "
                               f"Cmd(x,y,z,yaw):({forward_speed:.2f}, {sideways_speed:.2f}, {vertical_speed:.2f}, {yaw_speed:.2f})")
        
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