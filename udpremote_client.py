import socket
import keyboard
import time

UDP_IP = "192.168.137.205"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(command):
    sock.sendto(command.encode(), (UDP_IP, UDP_PORT))

try:
    while True:
        if keyboard.is_pressed('up'):
            send_command('UP')
            if keyboard.is_pressed('left'):
                send_command('UP_LEFT')
            elif keyboard.is_pressed('right'):
                send_command('UP_RIGHT')
            else:
                send_command('UP')
            
        elif keyboard.is_pressed('down'):
            send_command('DOWN')
            if keyboard.is_pressed('left'):
                send_command('DOWN_LEFT')
            elif keyboard.is_pressed('right'):
                send_command('DOWN_RIGHT')
            else:
                send_command('DOWN')
                
        elif keyboard.is_pressed('left'):
            send_command('LEFT')
            
        elif keyboard.is_pressed('right'):
            send_command('RIGHT') 

        else:
            send_command('STOP')

        time.sleep(0.0001)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    sock.close()