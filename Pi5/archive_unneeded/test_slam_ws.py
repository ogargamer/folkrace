import asyncio
import websockets
import json

async def test_ws():
    print("Connecting to Cartographer 2D Web Visualizer WebSocket (ws://10.43.8.42:8888/ws)...")
    async with websockets.connect("ws://10.43.8.42:8888/ws") as ws:
        for i in range(3):
            msg = await ws.recv()
            data = json.loads(msg)
            if data["type"] == "map":
                print(f"[{i+1}] MAP UPDATE: {data['width']}x{data['height']} cells ({data['width']*data['resolution']:.1f}m x {data['height']*data['resolution']:.1f}m) @ resolution {data['resolution']}m")
            elif data["type"] == "pose":
                print(f"[{i+1}] ROBOT POSE: X={data['x']:.2f}m, Y={data['y']:.2f}m, Yaw={data['yaw']*57.3:.1f}°, Path length={len(data['path'])}")

asyncio.run(test_ws())
