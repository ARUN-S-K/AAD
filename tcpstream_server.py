import cv2
import socket
import struct
import time
import msgpack

# Server setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ip = '192.168.137.205' 
server_port = 5005
server_socket.bind((server_ip, server_port))
server_socket.listen()

print("Server is listening...")

client_socket, client_address = server_socket.accept()
print(f"Client connected: {client_address}")

cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture frame from camera")
            break

        _, compressed_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])

        compressed_frame_list = compressed_frame.tolist()
        compressed_frame_data = msgpack.packb(compressed_frame_list)

        client_socket.sendall(struct.pack('!I', len(compressed_frame_data)))
        client_socket.sendall(compressed_frame_data)

        time.sleep(0.0001)

finally:
    cap.release()
    client_socket.close()
    server_socket.close()
