import urllib.request
import json

def test_reset_api():
    print("Sending POST request to http://10.43.8.42:8888/api/reset_map ...")
    req = urllib.request.Request("http://10.43.8.42:8888/api/reset_map", data=b"{}", headers={'Content-Type': 'application/json'})
    try:
        res = urllib.request.urlopen(req, timeout=3)
        body = json.loads(res.read().decode('utf-8'))
        print("Response from Reset API:", body)
    except Exception as e:
        print("Error calling reset API:", e)

if __name__ == "__main__":
    test_reset_api()
