
import os, sys, math
from controller import Supervisor

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
FOLKRACE_DIR = os.path.join(CONTROLLERS_DIR, 'folkrace_drive')
sys.path.insert(0, FOLKRACE_DIR)
from electronic_diff_4wd import ElectronicDifferential4WD

r = Supervisor()
ts = 20 # 50 Hz simulation
s_l = r.getDevice('servo left')
s_r = r.getDevice('servo right')
sn_l = r.getDevice('servo left sensor')
sn_r = r.getDevice('servo right sensor')
m_fl = r.getDevice('motor FL')
m_fr = r.getDevice('motor FR')
m_rl = r.getDevice('motor RL')
m_rr = r.getDevice('motor RR')
gyro = r.getDevice('gyro')

for dev in [m_fl, m_fr, m_rl, m_rr]:
    dev.setPosition(float('inf'))
    dev.setVelocity(0.0)

sn_l.enable(ts)
sn_r.enable(ts)
gyro.enable(ts)

node = r.getSelf()
tf = node.getField('translation')
rf = node.getField('rotation')

def reset():
    tf.setSFVec3f([0, 0, 0.028])
    rf.setSFRotation([0, 0, 1, 0])
    node.resetPhysics()
    for m in [m_fl, m_fr, m_rl, m_rr]: m.setVelocity(0.0)
    s_l.setPosition(0.0)
    s_r.setPosition(0.0)
    for _ in range(15): r.step(ts)

ediff = ElectronicDifferential4WD()

print('--- RWD PROPULSION ACKERMANN TEST (Rear driven, Front steering free) ---', flush=True)

test_angles = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0, -5.0, -10.0, -15.0, -20.0, -25.0, -30.0]

for steer_deg in test_angles:
    reset()
    rad = math.radians(steer_deg)
    speed = 1.50
    R = 0.1325 / math.tan(rad)
    ideal_yaw = speed / R
    act = ediff.update(target_v=speed, target_steer_rad=rad, gyro_z_rad_s=ideal_yaw)
    cmd_l = math.degrees(act['steer_left'])
    cmd_r = math.degrees(act['steer_right'])
    
    samples_l, samples_r, yaw_samples = [], [], []
    for step in range(35):
        s_l.setPosition(act['steer_left'])
        s_r.setPosition(act['steer_right'])
        # RWD drive
        m_rl.setVelocity(act['omega_RL'])
        m_rr.setVelocity(act['omega_RR'])
        r.step(ts)
        if step >= 15:
            samples_l.append(math.degrees(sn_l.getValue()))
            samples_r.append(math.degrees(sn_r.getValue()))
            yaw_samples.append(gyro.getValues()[2])
            
    avg_l = sum(samples_l) / len(samples_l)
    avg_r = sum(samples_r) / len(samples_r)
    osc_l = max(samples_l) - min(samples_l)
    osc_r = max(samples_r) - min(samples_r)
    avg_yaw = sum(yaw_samples) / len(yaw_samples)
    print(f'Steer {steer_deg:+5.1f} deg | Cmd: ({cmd_l:+5.2f}, {cmd_r:+5.2f}) | Meas: ({avg_l:+5.2f}, {avg_r:+5.2f}) | Osc: ({osc_l:4.2f}, {osc_r:4.2f}) deg | Yaw: {avg_yaw:+5.2f} (ideal {ideal_yaw:+5.2f})', flush=True)

r.simulationQuit(0)
