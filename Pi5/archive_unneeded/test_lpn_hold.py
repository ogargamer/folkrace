import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_gpio_and_run_ranging():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TESTING RESET & RANGING WITH ACCURATE LPn CONTROL")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

# Compile a tiny C helper to pulse and hold LPn (GPIO 17 on gpiochip4) HIGH
cat << 'EOF' > /tmp/pulse_lpn.c
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <string.h>

int main() {
    int chip_fd = open("/dev/gpiochip4", O_RDWR);
    if (chip_fd < 0) {
        perror("open gpiochip4");
        return 1;
    }

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = 17;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.lines = 1;
    strcpy(req.consumer_label, "lpn_ctrl");
    req.default_values[0] = 0; // start LOW

    if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        perror("GPIO_GET_LINEHANDLE_IOCTL");
        return 1;
    }

    printf("LPn -> LOW\n");
    usleep(50000); // 50ms LOW

    struct gpiohandle_data data;
    data.values[0] = 1;
    if (ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
        perror("GPIOHANDLE_SET_LINE_VALUES_IOCTL");
        return 1;
    }
    printf("LPn -> HIGH (Active)\n");
    usleep(100000); // 100ms HIGH

    // Now run ST ranging while keeping LPn HIGH
    printf("\n==> Launching STSW-IMG042 multi with LPn held HIGH...\n");
    chdir("/home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test");
    FILE *fp = popen("./multi", "w");
    if (fp) {
        fprintf(fp, "1\n");
        fflush(fp);
        pclose(fp);
    }

    close(req.fd);
    close(chip_fd);
    return 0;
}
EOF

gcc /tmp/pulse_lpn.c -o /tmp/pulse_lpn
/tmp/pulse_lpn
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_pulse_lpn.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_pulse_lpn.sh && /tmp/run_pulse_lpn.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_gpio_and_run_ranging()
