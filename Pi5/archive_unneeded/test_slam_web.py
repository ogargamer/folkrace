import urllib.request
import json
import time

def test_slam_web():
    print("Testing HTTP GET http://10.43.8.42:8080/ ...")
    try:
        req = urllib.request.urlopen("http://10.43.8.42:8080/", timeout=3)
        html = req.read().decode('utf-8')
        print(f">> HTTP GET Success: {len(html)} bytes received! Title: {'CARTOGRAPHER' in html}")
    except Exception as e:
        print("[HTTP Error]:", e)

if __name__ == "__main__":
    test_slam_web()
