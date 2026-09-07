import asyncio
import websockets
import json

async def test_ws():
    uri = "ws://10.43.8.42:8000/ws"
    print(f"Connecting to WebSocket: {uri} ...")
    async with websockets.connect(uri) as ws:
        print("Connected to WebSocket stream! Receiving 5 packets:")
        for i in range(5):
            msg = await ws.recv()
            data = json.loads(msg)
            print(f"  [{i+1}] Roll: {data['roll']:+6.2f}° | Pitch: {data['pitch']:+6.2f}° | Yaw: {data['yaw']:+6.2f}° | Accel: ({data['ax']:+5.2f}, {data['ay']:+5.2f}, {data['az']:+5.2f})")
    print(">> WebSocket Verification PASSED! <<")

if __name__ == "__main__":
    asyncio.run(test_ws())
