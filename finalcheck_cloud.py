import asyncio
import websockets

# Set up an event loop policy for better performance
asyncio.set_event_loop_policy(asyncio.DefaultEventLoopPolicy())

clients = set()

async def relay(websocket, path):
    clients.add(websocket)
    try:
        async for message in websocket:
            # Relay messages to other clients
            for client in clients:
                if client != websocket and client.open:
                    await client.send(message)
    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        clients.remove(websocket)

async def main():
    # Start WebSocket server on port 8765
    async with websockets.serve(relay, "0.0.0.0", 8765):
        print("WebSocket server running on ws://0.0.0.0:8765")
        await asyncio.Future()  # Run forever

# Run the relay server
asyncio.run(main())

