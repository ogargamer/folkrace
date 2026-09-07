import paramiko

s = paramiko.SSHClient()
s.set_missing_host_key_policy(paramiko.AutoAddPolicy())
s.connect('10.43.8.42', port=22, username='admin', password='l', timeout=3)
stdin, stdout, stderr = s.exec_command('python3 -c "for m in [\'websockets\', \'tornado\', \'flask\', \'aiohttp\', \'fastapi\', \'uvicorn\']: \n try:\n  __import__(m)\n  print(m, \'is INSTALLED\')\n except ImportError:\n  print(m, \'NOT installed\')"')
print(stdout.read().decode())
s.close()
