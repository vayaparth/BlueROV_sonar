#sonar_visualizer

import pygame
import math
import random
import rospy
from sensor_msgs.msg import LaserScan
import threading # For thread safety

# --- Pygame Init ---
pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 16)
center = (width / 2, height / 2)
radius = 250 # Display radius in pixels
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
DARKGREEN = (0, 150, 0)

# --- Data Shared Between ROS Callback and Pygame Loop ---
blips = []
data_lock = threading.Lock()
current_sweep_angle_rad = 0 # To draw the sweep line

# --- Pygame Functions ---
def scale_m_to_px(real_m, max_range_m=10.0, display_radius_px=250):
    # Clamp distance to max range before scaling
    real_m_clamped = min(real_m, max_range_m)
    return (real_m_clamped / max_range_m) * display_radius_px

def draw_radar():
    screen.fill(BLACK)
    # Draw concentric circles
    for i in range(50, int(radius) + 1, 50):
        pygame.draw.circle(screen, GREEN, center, i, 1)

    # Draw radial lines
    for deg in range(0, 360, 30):
        rad = math.radians(deg)
        x_line = center[0] + radius * math.cos(rad)
        y_line = center[1] + radius * math.sin(rad)
        pygame.draw.line(screen, DARKGREEN, center, (x_line, y_line), 1)

    # Draw the sweep line using the last known angle from the scan
    x_sweep = center[0] + radius * math.cos(current_sweep_angle_rad - math.pi/2) # Offset if needed
    y_sweep = center[1] + radius * math.sin(current_sweep_angle_rad - math.pi/2) # Offset if needed
    pygame.draw.line(screen, GREEN, center, (x_sweep, y_sweep), 2)

    # Draw blips (acquire lock to safely access list)
    with data_lock:
        for blip in blips:
            pygame.draw.circle(screen, GREEN, blip, 3) # Smaller blips

# --- ROS Callback ---
def laser_callback(msg):
    global blips, current_sweep_angle_rad # Allow modification of global variables

    # --- Obstacle Detection Logic (Example - consider moving to dedicated node) ---
    danger_threshold_m = 0.5 # e.g., 50 cm
    # Example: Check angles roughly corresponding to -15 to +15 degrees
    # Need to calculate indices based on angle_min, angle_max, angle_increment
    # angle = msg.angle_min + i * msg.angle_increment
    # For Ping360, 0 gradians might be forward, which is 0 radians.
    # Let's assume 0 rad is forward. +/- 15 deg is approx +/- 0.26 rad
    obstacle_found = False
    try:
        center_index = int((-msg.angle_min) / msg.angle_increment) # Index for 0 rad
        angle_offset_indices = int(0.26 / msg.angle_increment) # Indices for +/- 15 deg
        start_idx = max(0, center_index - angle_offset_indices)
        end_idx = min(len(msg.ranges) -1, center_index + angle_offset_indices)

        for i in range(start_idx, end_idx + 1):
            if msg.range_min < msg.ranges[i] < danger_threshold_m:
                obstacle_found = True
                rospy.logwarn_throttle(1.0, f"OBSTACLE DETECTED at index {i}, distance {msg.ranges[i]:.2f}m")
                # In a dedicated node, publish a message here
                break # Stop checking once one is found in the zone
    except Exception as e:
        rospy.logerr_throttle(5.0, f"Error calculating indices: {e}")


    # --- Update Visualization Data ---
    new_blips = []
    last_valid_angle = 0 # Keep track for sweep line
    for i, distance in enumerate(msg.ranges):
        if msg.range_min < distance < msg.range_max:
            current_angle_rad = msg.angle_min + i * msg.angle_increment
            last_valid_angle = current_angle_rad

            # --- Calculate pixel position ---
            # Adjust angle: Often ROS 0 rad (forward) needs mapping to Pygame's 0 rad (right)
            # Or adjust based on sonar mounting. Common offset: -math.pi/2 makes 0 rad point UP.
            display_angle_rad = current_angle_rad - (math.pi / 2)

            blip_dist_px = scale_m_to_px(distance, max_range_m=msg.range_max, display_radius_px=radius)
            blip_x = int(center[0] + blip_dist_px * math.cos(display_angle_rad))
            blip_y = int(center[1] + blip_dist_px * math.sin(display_angle_rad))
            new_blips.append((blip_x, blip_y))

    # Safely update the shared blips list and sweep angle
    with data_lock:
        blips = new_blips
        current_sweep_angle_rad = last_valid_angle


# --- Main Program ---
if __name__ == '__main__':
    try:
        rospy.init_node('sonar_visualizer', anonymous=True)
        # Subscribe to the LaserScan topic (adjust topic name if needed)
        rospy.Subscriber("/scan", LaserScan, laser_callback)
        rospy.loginfo("Sonar Visualizer Node Started")

        running = True
        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            draw_radar()
            pygame.display.flip()
            clock.tick(30) # Limit Pygame framerate

        # No need for rospy.spin() if the loop checks rospy.is_shutdown()

    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        pygame.quit()