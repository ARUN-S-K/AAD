import asyncio
import websockets
import cv2
import time
import base64
import RPi.GPIO as GPIO
from queue import Queue
from threading import Thread
import numpy as np
import math
import sys
# GPIO Setup
GPIO.setwarnings(False)
GPIO.cleanup()


ENA = 12    # PWM pin for Motor 1 speed control
IN1 = 23    # Direction pin 1 for Motor 1 (forward)
IN2 = 22    # Direction pin 2 for Motor 1 (backward)
ENB = 13    # PWM pin for Motor 2 speed control
IN3 = 17    # Direction pin 1 for Motor 2 (left turn)
IN4 = 27    # Direction pin 2 for Motor 2 (right turn)

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

pwm_motor1 = GPIO.PWM(ENA, 1000)  
pwm_motor2 = GPIO.PWM(ENB, 1000)  
pwm_motor1.start(0)
pwm_motor2.start(0)

def move_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor1.ChangeDutyCycle(speed)

def move_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor1.ChangeDutyCycle(speed)

def turn_left(speed):
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_motor2.ChangeDutyCycle(speed)

def turn_right(speed):
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_motor2.ChangeDutyCycle(speed)

def stop():
    pwm_motor1.ChangeDutyCycle(0)
    pwm_motor2.ChangeDutyCycle(0) 

# Autonomous Driving Flag
autonomous_mode = False

def detect_edges(frame):
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range for red color (two ranges in HSV space)
    lower_red1 = np.array([0, 50, 50], dtype="uint8")  # First range (low red)
    upper_red1 = np.array([10, 255, 255], dtype="uint8")  # Upper bound for low red
    lower_red2 = np.array([170, 50, 50], dtype="uint8")  # Second range (high red)
    upper_red2 = np.array([180, 255, 255], dtype="uint8")  # Upper bound for high red

    # Create two masks for the two red ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine the masks
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Apply Canny edge detection
    edges = cv2.Canny(mask, 50, 100)
    
    return edges


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    
    cropped_edges = cv2.bitwise_and(edges, mask)
    cv2.imshow("roi",cropped_edges)
    
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10  
    
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    
    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue
            
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    
    slope, intercept = line
    
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    
    if slope == 0:
        slope = 0.1
        
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)
    
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    
    return line_image


def display_heading_line(frame, pwm_motor2_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    pwm_motor2_angle_radian = pwm_motor2_angle / 180.0 * math.pi
    
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(pwm_motor2_angle_radian))
    y2 = int(height / 2)
    
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    
    return heading_image

def get_pwm_motor2_angle(frame, lane_lines):
    
    height,width,_ = frame.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
        
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    pwm_motor2_angle = angle_to_mid_deg + 90
    
    return pwm_motor2_angle

# Autonomous Model (White Paper Detection)
def process_frame(frame):
    """Process frame to detect lanes and compute the angle for navigation."""
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame, line_segments)

    if lane_lines:
        pwm_motor2_angle = get_pwm_motor2_angle(frame, lane_lines)
        return pwm_motor2_angle
    return None

# Video Streaming
async def send_video(frame_queue, websocket):
    cap = cv2.VideoCapture(0)  # USB camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Encode frame as JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        frame_data = base64.b64encode(buffer).decode('utf-8')
        timestamp = time.time()
        message = f"{timestamp}|{frame_data}"

        # Send frame to WebSocket server
        try:
            await websocket.send(message)
            frame_queue.put(frame)  # Add frame to queue for autonomous processing
            await asyncio.sleep(0.0001)  # ~30 FPS
        except websockets.ConnectionClosed:
            break

    cap.release()

# Command Receiver
async def receive_commands(websocket):
    global autonomous_mode
    while True:
        try:
            message = await websocket.recv()
            if message == "ACTIVATE_AUTONOMOUS":
                autonomous_mode = True
                print("Switched to Autonomous Mode")
            elif message == "ACTIVATE_REMOTE":
                autonomous_mode = False
                print("Switched to Remote Control Mode")
            elif not autonomous_mode:
                # Handle remote control commands
                if message == "UP":
                    move_forward(50)
                elif message == "DOWN":
                    move_backward(50)
                elif message == "LEFT":
                    # Example: Add left turn logic
                    turn_left(50)
                elif message == "RIGHT":
                    # Example: Add right turn logic
                    turn_right(50)
                elif message == "STOP":
                    stop()
        except websockets.ConnectionClosed:
            break

# Autonomous Driving Loop
def autonomous_loop(frame_queue, websocket):
    """Autonomous driving loop to process frames and control motors."""
    global autonomous_mode
    kp = 0.4
    kd = kp * 0.65
    last_time = time.time()
    last_error = 0
    base_speed = 25

    while True:
        if autonomous_mode and not frame_queue.empty():
            frame = frame_queue.get()
            pwm_motor2_angle = process_frame(frame)

            if pwm_motor2_angle is not None:
                deviation = pwm_motor2_angle - 90
                error = abs(deviation)
                now = time.time()
                dt = now - last_time

                # PD control for steering
                derivative = kd * (error - last_error) / dt
                proportional = kp * error
                PD = int(base_speed + derivative + proportional)
                spd = min(abs(PD), 75)  # Cap speed to 75

                # Steering logic
                if -5 <= deviation <= 5:
                    GPIO.output(IN3, GPIO.LOW)
                    GPIO.output(IN4, GPIO.LOW)
                    pwm_motor2.ChangeDutyCycle(0)  # Stop turning
                elif deviation > 5:
                    GPIO.output(IN3, GPIO.LOW)
                    GPIO.output(IN4, GPIO.HIGH)
                    pwm_motor2.ChangeDutyCycle(100)
                elif deviation < -5:
                    GPIO.output(IN3, GPIO.HIGH)
                    GPIO.output(IN4, GPIO.LOW)
                    pwm_motor2.ChangeDutyCycle(100)

                # Move forward
                move_forward(spd)
                print(f"Autonomous: Moving forward with speed {spd}, deviation {deviation}")

                last_error = error
                last_time = now
            else:
                stop()
                print("Autonomous: No path detected. Requesting remote control.")
                asyncio.run(websocket.send("REQUEST_REMOTE"))
                autonomous_mode = False

# Main Function
async def main():
    uri = "ws://4.240.96.209:8765"  # Cloud server
    frame_queue = Queue()

    async with websockets.connect(uri) as websocket:
        # Start video streaming and command receiver concurrently
        video_task = asyncio.create_task(send_video(frame_queue, websocket))
        command_task = asyncio.create_task(receive_commands(websocket))

        # Start autonomous driving loop in a separate thread
        autonomous_thread = Thread(target=autonomous_loop, args=(frame_queue, websocket))
        autonomous_thread.start()

        await asyncio.gather(video_task, command_task)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    GPIO.cleanup()
