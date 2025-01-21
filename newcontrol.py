import asyncio
import websockets
import time
import keyboard  # Install using `pip install keyboard`

async def send_commands():
    uri = "ws://4.240.96.209:8765"  # Replace with your WebSocket server's URI
    async with websockets.connect(uri) as websocket:
        print("Connected. Sending commands.")
        while True:
            try:
                command = "STOP"  # Default command

                # Check for keyboard inputs
                if keyboard.is_pressed('up'):
                    if keyboard.is_pressed('left'):
                        command = "UP_LEFT"
                    elif keyboard.is_pressed('right'):
                        command = "UP_RIGHT"
                    else:
                        command = "UP"
                elif keyboard.is_pressed('down'):
                    if keyboard.is_pressed('left'):
                        command = "DOWN_LEFT"
                    elif keyboard.is_pressed('right'):
                        command = "DOWN_RIGHT"
                    else:
                        command = "DOWN"
                elif keyboard.is_pressed('left'):
                    command = "LEFT"
                elif keyboard.is_pressed('right'):
                    command = "RIGHT"

                # Append timestamp to the command
                timestamp = time.time()
                message = f"{command}|{timestamp}"

                # Send the command with timestamp
                await websocket.send(message)

                # Log the sent command
                print(f"Sent: {command}, Timestamp: {timestamp}")

                # Small delay to avoid high CPU usage
                await asyncio.sleep(0.00001)  # 10ms delay
            except Exception as e:
                print(f"Error: {e}")
                break

asyncio.run(send_commands())
