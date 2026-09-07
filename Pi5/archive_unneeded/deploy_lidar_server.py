import paramiko
import os
import sys

sys.stdout.reconfigure(encoding='utf-8')

def deploy_and_run():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    print(f"Connecting to Pi 5 at {ip}...")
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
    print("Connected!")

    # Create remote web directory
    ssh.exec_command("mkdir -p /home/admin/web")

    sftp = ssh.open_sftp()
    
    # 1. Upload lidar_server.py
    local_server = r"c:\Users\nurmi\Documents\Da Winna\Pi5\lidar_server.py"
    sftp.put(local_server, "/home/admin/lidar_server.py")
    print("Uploaded lidar_server.py")

    # 2. Upload index.html
    local_html = r"c:\Users\nurmi\Documents\Da Winna\Pi5\web\index.html"
    sftp.put(local_html, "/home/admin/web/index.html")
    print("Uploaded web/index.html")

    sftp.close()

    # Kill any existing server instance
    ssh.exec_command("pkill -f lidar_server.py 2>/dev/null || true")

    # Start lidar_server.py in background via nohup
    print("Starting MS200 Visualizer Server in background...")
    stdin, stdout, stderr = ssh.exec_command("nohup python3 /home/admin/lidar_server.py > /home/admin/lidar_server.log 2>&1 &", get_pty=True)
    
    import time
    time.sleep(1.5)

    # Check server log
    stdin, stdout, stderr = ssh.exec_command("cat /home/admin/lidar_server.log | head -n 20")
    print("--- Server Output Log ---")
    print(stdout.read().decode())

    ssh.close()
    print("Deployment completed successfully!")

if __name__ == "__main__":
    deploy_and_run()
