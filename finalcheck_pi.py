import asyncio
import websockets
import cv2
import time
import base64
import RPi.GPIO as GPIO
from queue import Queue
from threading import Thread

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

# Autonomous Model (White Paper Detection)
def process_frame(frame):
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = (0, 0, 200)
    upper_white = (180, 40, 255)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # Detect large white regions
            return True
    return False

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
            await asyncio.sleep(0.033)  # ~30 FPS
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
    global autonomous_mode
    while True:
        if autonomous_mode:
            if not frame_queue.empty():
                frame = frame_queue.get()
                if process_frame(frame):
                    move_forward(50)
                    print("Autonomous: Moving forward")
                else:
                    stop()
                    print("Autonomous: No path detected")
                    asyncio.run(websocket.send("REQUEST_REMOTE"))  # Request remote control
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
