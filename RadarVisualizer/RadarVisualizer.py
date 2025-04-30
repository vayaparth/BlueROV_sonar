#!/usr/bin/env python3
# ==========================================
# Combined Pygame ROS Node (Middle Point Radar) - CORRECTED
# ==========================================
import pygame
import rospy
from sensor_msgs.msg import LaserScan # Message type for /sonar/scan
import math
import threading
import sys
from std_msgs.msg import String

def chatter_callback(message):
    """
    This function is called each time a message is received on the '/chatter' topic.
    'message' is an object containing the data. For String messages, the data is in the '.data' field.
    """
    rospy.loginfo("I heard: [%s]", message.data) # Log using ROS logger (visible with rosout)
    # Alternatively, use standard print:
    # print(f"Received on /chatter: {message.data}")
    # You could also print the whole message object:
    # print("Full message object:")
    # print(message)



# --- Configuration ---
NODE_NAME = 'pygame_radar_node'
# !! IMPORTANT: Set this to your actual LaserScan topic name !!
LASERSCAN_TOPIC = '/sonar/scan' # Confirmed topic name
print(LASERSCAN_TOPIC)
# Pygame Settings
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
DISPLAY_RADIUS_PX = 350 # Pixel radius for radar display
# Max distance (meters) to map to DISPLAY_RADIUS_PX. Adjust based on sensor/needs.
MAX_DISPLAY_RANGE_M = 5.0 # Example: 5 meters max range shown
FPS = 30 # Target frames per second
BACKGROUND_COLOR = (0, 30, 0) # Dark green
GRID_COLOR = (0, 100, 0) # Lighter green for grid/labels
BLIP_COLOR_START = (180, 255, 180) # Base color for blips (brightness varies)
SWEEP_COLOR = (100, 255, 100) # Color of the sweep line

# --- Global variables for ROS data ---
data_lock = threading.Lock() # Lock for thread-safe data access
# Dictionary to hold the latest data received from the callback
shared_data = {
    'distance': None,   # Stores latest middle distance (meters)
    'angle_deg': None, # Stores latest middle angle (degrees, sensor frame)
    'intensity': None  # Stores latest middle intensity (raw sensor value)
}

# --- Helper function to scale intensity ---
def scale_intensity_for_display(raw_intensity):
    """
    Scales raw sensor intensity to a display brightness value (e.g., 50-255).
    Needs adjustment based on your specific sensor's intensity range.
    """
    if raw_intensity is None:
        return 50 # Default low brightness if no intensity/invalid

    # !!! IMPORTANT: Adjust MIN_INTENSITY and MAX_INTENSITY based on your sensor's typical output !!!
    MIN_INTENSITY = 0.0   # Example: Assume minimum intensity is 0
    MAX_INTENSITY = 1000.0 # Example: Assume maximum intensity is 1000

    # Clamp the intensity to the expected range to handle outliers
    clamped_intensity = max(MIN_INTENSITY, min(raw_intensity, MAX_INTENSITY))

    # Normalize the clamped intensity to a 0.0 - 1.0 range
    # Add epsilon to denominator to avoid division by zero if MAX == MIN
    normalized_intensity = (clamped_intensity - MIN_INTENSITY) / max(1e-6, (MAX_INTENSITY - MIN_INTENSITY))

    # Scale the normalized value to a display brightness range (e.g., 50 to 255)
    # This ensures even low intensities are slightly visible (brightness >= 50)
    display_brightness = 50 + (normalized_intensity * 205) # 50 + (0..1 * 205) = 50..255

    return int(display_brightness)


# --- ROS Callback Function (Processes incoming LaserScan messages) ---
def sonar_callback(msg):
    """
    Processes LaserScan message to get the middle point's distance, angle, and intensity.
    Reads necessary parameters from the 'msg' object and updates shared_data.
    """
    global shared_data # Allow modifying the global shared_data dictionary

    # Read necessary parameters directly from the message object ('msg')
    ranges = msg.ranges #we get 111 values here in floating point

    intensities = msg.intensities # Read intensities array
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    range_min = msg.range_min
    print("my min range is",range_min)
    range_max = msg.range_max
    print("My range max:", range_max)

    num_ranges = len(ranges)
    print("length")
    print(num_ranges)
    num_intensities = len(intensities) # Get length of intensities array
    
    # Basic check: If no ranges, nothing to process
    if num_ranges == 0:
        return

    # Sanity check: Ensure ranges and intensities arrays have the same size
    if num_ranges != num_intensities:
        # Log warning occasionally if sizes mismatch (indicates faulty message)
        rospy.logwarn_throttle(10, f"Ranges ({num_ranges}) and intensities ({num_intensities}) length mismatch!")
        return

    # Calculate the index of the middle point in the arrays
    middle_index = num_ranges // 2
    print("middle index=")
    print(middle_index)
    # Get the distance and intensity reading for the middle point
    distance_m = ranges[middle_index]
    print("my distance at middle point",distance_m)
    intensity_val = intensities[middle_index] # Get the corresponding intensity
    print("intesity_value at this point",intensity_val)
    # --- Validate the distance reading ---
    # Check if the distance is a valid number (not inf/nan) AND within the sensor's physical range limits
    if not (math.isfinite(distance_m) or distance_m < range_min or distance_m > range_max):
        # Invalid reading, set all corresponding shared data to None for this cycle
        valid_distance = 1
        calculated_angle_deg = 0.5
        valid_intensity = 35
        # Log invalid readings occasionally for debugging
        rospy.logdebug_throttle(2, f"Middle point distance invalid: {distance_m}")
    else:
        # Valid distance found
        valid_distance = distance_m
        valid_intensity = intensity_val # Intensity is considered valid if distance is

        # --- Calculate the angle for the middle point ---
        # Formula: angle = start_angle + index * angle_step
        angle_rad = angle_min + (middle_index * angle_increment)
        calculated_angle_deg = math.degrees(angle_rad) # Convert radians to degrees

    # --- Update shared data structure (thread-safe using lock) ---
    # This makes the calculated data available to the Pygame loop
    with data_lock:
        shared_data['distance'] = valid_distance
        shared_data['angle_deg'] = calculated_angle_deg
        shared_data['intensity'] = valid_intensity # Store intensity
        print(shared_data)

# --- Helper Functions (For Pygame Display Logic) ---
def scale_meters_to_pixels(distance_m, display_radius_px, max_display_range_m):
    """Scales distance in meters to pixels relative to the display radius."""
    if distance_m is None or max_display_range_m <= 0:
        return None # Cannot scale if distance is invalid or max range is zero/negative
    # Ensure distance doesn't exceed the visualization's max range for scaling
    display_distance_m = min(distance_m, max_display_range_m)
    # Calculate scale factor: pixels per meter
    scale = display_radius_px / max_display_range_m
    return int(display_distance_m * scale)

def map_ros_angle_to_screen(angle_deg):
    """ Maps ROS angle (degrees, 0=sensor forward/X-axis)
        to the screen's coordinate system angle (radians, 0=Up/North, increases CW)
        suitable for drawing the radar display.
    """
    if angle_deg is None:
        return None
    # Convert ROS angle: ROS 0 (forward) should become Screen 90 degrees (Up).
    # ROS positive angle (CCW) should become Screen negative angle change (CW).
    screen_angle_deg = -angle_deg + 90
    print("angle degree:", screen_angle_deg)
    return math.radians(screen_angle_deg) # Return radians as math functions use them

def draw_radar(screen, center, display_radius_px, sweep_angle_deg, blips, font):
    """Draws the static radar display elements (grid, labels) and dynamic elements (sweep, blips)."""
    # Fill background
    screen.fill(BACKGROUND_COLOR)

    # Draw range rings (grid)
    num_rings = 5 # Example: Draw 5 concentric rings
    for i in range(1, num_rings + 1):
        radius = int(display_radius_px * (i / num_rings))
        pygame.draw.circle(screen, GRID_COLOR, center, radius, 1) # Draw the ring
        # Draw range labels near the rings
        range_m = (i / num_rings) * MAX_DISPLAY_RANGE_M
        label = font.render(f"{range_m:.1f}m", True, GRID_COLOR)
        # Position label slightly outside the ring, centered horizontally
        screen.blit(label, (center[0] + 5, center[1] - radius - font.get_height()))

    # Draw crosshairs (horizontal and vertical lines)
    pygame.draw.line(screen, GRID_COLOR, (center[0] - display_radius_px, center[1]), (center[0] + display_radius_px, center[1]), 1)
    pygame.draw.line(screen, GRID_COLOR, (center[0], center[1] - display_radius_px), (center[0], center[1] + display_radius_px), 1)

    # Draw rotating sweep line
    sweep_rad = math.radians(-sweep_angle_deg + 90) # Convert sweep angle for drawing
    end_x = center[0] + display_radius_px * math.cos(sweep_rad)
    # Use negative sin because Pygame's Y-axis increases downwards
    end_y = center[1] - display_radius_px * math.sin(sweep_rad)
    pygame.draw.line(screen, SWEEP_COLOR, center, (int(end_x), int(end_y)), 2)

    # Draw and decay blips
    new_blips = [] # List to hold blips for the *next* frame
    for blip in blips:
        x, y, current_brightness = blip # Unpack blip data

        # Ensure brightness is within valid range for color calculation
        brightness_val = max(0, min(int(current_brightness), 255))

        # Only draw and keep blip if its brightness is still sufficiently high
        if brightness_val > 10:
            # Calculate blip color based on current brightness (scales the base color)
            # Ensures color components stay within the valid 0-255 range
            blip_color = tuple(max(0, min(255, int(c * (brightness_val/255.0)))) for c in BLIP_COLOR_START)

            # Draw the blip as a small circle
            pygame.draw.circle(screen, blip_color, (x, y), 3)

            # Calculate decayed brightness for the next frame
            # Adjust decay factor (e.g., 0.95) to control how fast blips fade
            decayed_brightness = current_brightness * 0.95
            # Add the blip back to the list for the next frame with reduced brightness
            new_blips.append([x, y, decayed_brightness])

    return new_blips # Return the updated list of blips for the next iteration


# --- Main Execution Block (Runs when script is executed) ---
if __name__ == '__main__':
    # 1. Initialize Pygame components
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"ROS Radar Display ({LASERSCAN_TOPIC})")
    clock = pygame.time.Clock() # Pygame clock for controlling FPS
    font = pygame.font.SysFont("Arial", 14) # Font for drawing labels

    blips = [] # Initialize an empty list to store active blips: [x, y, current_brightness]
    sweep_angle = 0 # Initialize the sweep line angle in degrees

    try:
        # 2. Initialize this script as a ROS Node
        rospy.init_node(NODE_NAME, anonymous=True) # anonymous=True allows multiple instances
        rospy.loginfo(f"{NODE_NAME} started, subscribing to {LASERSCAN_TOPIC}")

        # 3. Create a ROS Subscriber
        # This connects the specified topic to the 'sonar_callback' function.
        # ROS handles calling the callback in a separate thread when messages arrive.
        rospy.Subscriber(LASERSCAN_TOPIC, LaserScan, sonar_callback)

        # 4. Pygame Main Loop (Runs continuously)
        running = True
        while running and not rospy.is_shutdown(): # Loop continues if Pygame is running AND ROS is okay
            # --- Control frame rate ---
            # Get time elapsed since last frame in milliseconds (for smooth animations)
            dt_ms = clock.tick(FPS)

            # --- Pygame Event Handling ---
            # Check for user input events (like closing the window)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False # Set flag to exit the main loop

            # --- Get Latest Data from ROS Callback (using thread-safe lock) ---
            current_distance = None
            current_angle_deg = None
            current_intensity = None # Variable to hold fetched intensity
            with data_lock: # Acquire lock to safely read shared data
                current_distance = shared_data['distance']
                current_angle_deg = shared_data['angle_deg']
                current_intensity = shared_data['intensity'] # Fetch intensity
                # Optional: Clear shared data after reading if desired
                # shared_data['distance'] = None
                # shared_data['angle_deg'] = None
                # shared_data['intensity'] = None
            # Lock is automatically released here

            # --- Process Data & Add New Blips ---
            # Check if valid data was received from the callback
            if current_distance is not None and current_angle_deg is not None:
                # Convert distance (meters) to pixel radius for display
                distance_px = scale_meters_to_pixels(current_distance, DISPLAY_RADIUS_PX, MAX_DISPLAY_RANGE_M)
                print("my distancepx",distance_px)
                # Only proceed if scaling resulted in a valid pixel distance
                if distance_px is not None and distance_px > 0:
                    # Convert ROS angle (sensor frame) to screen angle (radians) for calculations
                    angle_rad_screen = map_ros_angle_to_screen(current_angle_deg)
                    print("my angle",angle_rad_screen)
                    if angle_rad_screen is not None:
                        # Calculate blip's screen (x, y) coordinates using trigonometry
                        x = int(CENTER[0] + distance_px * math.cos(angle_rad_screen))
                        # Use negative sine for Y because Pygame's Y increases downwards
                        y = int(CENTER[1] - distance_px * math.sin(angle_rad_screen))
                        print("my x and my y",x, y)
                        # --- Use Intensity for Initial Brightness ---
                        # Scale the raw sensor intensity to a display brightness value
                        initial_brightness = scale_intensity_for_display(current_intensity)
                        # Add the new blip to the list with its calculated position and initial brightness
                        blips.append([x, y, initial_brightness])
                        print("my blips",blips)
            # --- Update Sweep Angle ---
            # Rotate sweep line smoothly based on elapsed time (e.g., 180 degrees per second)
            sweep_angle = (sweep_angle + (180.0 * (dt_ms / 1000.0))) % 360 # Degrees per second * seconds elapsed

            # --- Draw Everything on Screen ---
            # Call the main drawing function which handles background, grid, sweep, and blips
            # It also returns the updated list of blips (after decay)
            blips = draw_radar(screen, CENTER, DISPLAY_RADIUS_PX, sweep_angle, blips, font)

            # --- Update Display ---
            # Make everything drawn visible on the screen
            pygame.display.flip()

        # --- End of main while loop ---

    except rospy.ROSInterruptException:
        # Handle Ctrl+C gracefully during ROS operation
        rospy.loginfo("ROS node interrupted (Ctrl+C).")
    except Exception as e:
        # Log any other unexpected errors during execution
        rospy.logerr(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc() # Print detailed error information to the console
    finally:
        # 5. Cleanup Resources
        pygame.quit() # Close the Pygame window and release resources
        rospy.loginfo("Pygame radar display shut down.")
        sys.exit() # Exit the script