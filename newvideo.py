import asyncio
import websockets
import cv2
import time
import base64
import numpy as np

async def receive_video():
    uri = "ws://4.240.96.209:8765"  # Replace with your WebSocket server's URI
    async with websockets.connect(uri) as websocket:
        print("Connected to video stream.")

        while True:
            try:
                # Receive frame with timestamp
                message = await websocket.recv()
                if "|" in message:
                    timestamp, frame_data = message.split("|")
                    timestamp = float(timestamp)

                    # Decode the base64 frame
                    frame_bytes = base64.b64decode(frame_data)
                    np_frame = np.frombuffer(frame_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(np_frame, cv2.IMREAD_COLOR)

                    # Calculate latency
                    latency = time.time() - timestamp
                    print(f"Received frame, Latency: {latency:.4f}s")

                    # Display the frame
                    cv2.imshow("Video Stream", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    print("Invalid message received.")
            except websockets.ConnectionClosed as e:
                print(f"Connection closed: {e}")
                break

        cv2.destroyAllWindows()

try:
    asyncio.run(receive_video())
except KeyboardInterrupt:
    print("Video receiving stopped by user.")
