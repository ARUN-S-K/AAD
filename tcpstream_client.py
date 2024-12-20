import cv2
import socket
import struct
import msgpack
import numpy as np


client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ip = '192.168.137.205'
server_port = 5005

client_socket.connect((server_ip, server_port))
print("Client connected to server")

while True:
    try:
        packed_size = client_socket.recv(4)
        if not packed_size:
            print("No size data received. Exiting...")
            break
        data_size = struct.unpack('!I', packed_size)[0]
        data = b''
        while len(data) < data_size:
            packet = client_socket.recv(min(4096, data_size - len(data)))
            if not packet:
                print("Incomplete frame data received. Exiting...")
                break
            data += packet
        compressed_frame_list = msgpack.unpackb(data, raw=False)
        compressed_frame = np.array(compressed_frame_list, dtype=np.uint8)
        frame = cv2.imdecode(compressed_frame, cv2.IMREAD_COLOR)       
        if frame is None:
            print("Error: Decoded frame is empty")
            continue
        cv2.imshow("Video Stream", frame)
        if cv2.waitKey(1) == ord('q'):
            break
    except (EOFError, ConnectionResetError, msgpack.UnpackValueError) as e:
        print(f"Error receiving data: {e}")
        break
client_socket.close()
cv2.destroyAllWindows()
