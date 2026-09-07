import urllib.request

try:
    req = urllib.request.urlopen("http://10.43.8.42:8888/", timeout=3)
    html = req.read().decode('utf-8')
    print(">> HTTP GET 8888 SUCCESS! Length:", len(html))
    print("Contains 'CARTOGRAPHER':", 'CARTOGRAPHER' in html)
except Exception as e:
    print("[Error]:", e)
