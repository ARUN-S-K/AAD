import RPi.GPIO as GPIO
import socket
import time

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


UDP_IP = "0.0.0.0"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

try:
    while True:
        data, addr = sock.recvfrom(1024) 
        command = data.decode()
        
        if command == 'UP':
            move_forward(100)
            pwm_motor2.ChangeDutyCycle(0)
        elif command == 'DOWN':
            move_backward(100)
            pwm_motor2.ChangeDutyCycle(0)
        elif command == 'LEFT':
            turn_left(100)
            pwm_motor1.ChangeDutyCycle(0)
        elif command == 'RIGHT':
            turn_right(100)
            pwm_motor1.ChangeDutyCycle(0)
        elif command == 'UP_LEFT':
            move_forward(100)
            turn_left(100)
        elif command == 'UP_RIGHT':
            move_forward(100)
            turn_right(100)
        elif command == 'DOWN_LEFT':
            move_backward(100)
            turn_left(100)
        elif command == 'DOWN_RIGHT':
            move_backward(100)
            turn_right(100)
        elif command == 'STOP':
            stop()
        else:
            print(f"Invalid command: {command}")

        time.sleep(0.0001)

except KeyboardInterrupt:
    stop()  
finally:
    pwm_motor1.stop()
    pwm_motor2.stop()
    GPIO.cleanup()
    sock.close()
