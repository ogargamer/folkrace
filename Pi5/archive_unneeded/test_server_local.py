import urllib.request
import json
import time

def test_server():
    print("Testing HTTP GET http://10.43.8.42:8000/ ...")
    try:
        req = urllib.request.urlopen("http://10.43.8.42:8000/", timeout=3)
        html = req.read().decode('utf-8')
        print(f">> HTTP GET Success: {len(html)} bytes received! Title present: {'BNO08X' in html}")
    except Exception as e:
        print("[HTTP Error]:", e)

    print("\nTesting POST /api/set_rate?hz=50 ...")
    try:
        data = b""
        req = urllib.request.Request("http://10.43.8.42:8000/api/set_rate?hz=50", data=data, method="POST")
        resp = urllib.request.urlopen(req, timeout=3)
        print(">> API Response:", resp.read().decode('utf-8'))
    except Exception as e:
        print("[API Error]:", e)

if __name__ == "__main__":
    test_server()
