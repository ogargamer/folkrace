import asyncio
import websockets
import json

async def test_ws():
    print("Connecting to ws://10.43.8.42:8080/ws ...")
    async with websockets.connect("ws://10.43.8.42:8080/ws") as ws:
        print("Connected to WebSocket!")
        for i in range(5):
            msg = await ws.recv()
            data = json.loads(msg)
            pts = data.get("points", [])
            tel = data.get("telemetry", {})
            print(f"[WebSocket Frame #{i+1}] Points: {len(pts)} | Hz: {tel.get('hz')} | Pts/Sec: {tel.get('points_per_sec')} | Min Dist: {tel.get('min_dist')} mm | Max Dist: {tel.get('max_dist')} mm")
            if pts:
                print(f"   -> Sample Point (Angle: {pts[0][0]}°, Dist: {pts[0][1]} mm, Conf: {pts[0][2]})")

if __name__ == "__main__":
    asyncio.run(test_ws())
