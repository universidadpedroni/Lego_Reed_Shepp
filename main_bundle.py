# ============================================================================
# main_bundle.py - GENERADO por SMC/make_bundle.py (2026-11-20 17:45). NO EDITAR A MANO:
# editar los .py originales y volver a correr el script.
# ============================================================================
import umath
import gc
from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor
from usys import stdin
from uselect import poll
from pybricks.parameters import Color
from pybricks.tools import wait
L_car = 24.0; psi_max = 20.0; v = 40.0; r_turn_min = 80.0; max_drive_power = 100; max_angle_power = 100; KP_POSITION = 50.0; KI_POSITION = 0.01; KP_STEERING = 50.0; KI_STEERING = 0.01; FORWARD = 1; BACKWARD = -1; LEFT = -1; RIGHT = 1; STRAIGHT = 0
def denormalize_distance_in_path(path, r_turn_min):

    for p in path: p['distance'] = p['distance'] * r_turn_min
    return path
def normalize_start_and_end_point(start_point_original, end_point_original, r_turn_min):
 start_point = [0, 0, 0]; end_point = [0, 0, 0]; start_point[0] = start_point_original[0] / r_turn_min; start_point[1] = start_point_original[1] / r_turn_min; start_point[2] = umath.radians(start_point_original[2]); end_point[0] = end_point_original[0] / r_turn_min; end_point[1] = end_point_original[1] / r_turn_min; end_point[2] = umath.radians(end_point_original[2]); return (start_point, end_point)
def R(x, y):
 u = umath.sqrt(x ** 2 + y ** 2); t = umath.atan2(y, x); return (u, t)
def M(theta):

    theta = umath.fmod(theta, 2 * umath.pi)
    if theta < -umath.pi: theta += 2 * umath.pi
    elif theta >= umath.pi: theta -= 2 * umath.pi
    return theta
def create_path_element(distance, steering, gear):

    if distance < 0: gear = -gear
    return {'distance': abs(distance), 'steering': steering, 'gear': gear}
def change_of_basis(start_point, end_point):
 (x, y, theta) = start_point; (x_end, y_end, theta_end) = end_point; dx = x_end - x; dy = y_end - y; dtheta = M(theta_end - theta); x = dx * umath.cos(theta) + dy * umath.sin(theta); y = -dx * umath.sin(theta) + dy * umath.cos(theta); return (x, y, dtheta)
def path1(x, y, phi):
 path = []; (u, t) = R(x - umath.sin(phi), y - 1 + umath.cos(phi)); v = M(phi - t); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, STRAIGHT, FORWARD)); path.append(create_path_element(v, LEFT, FORWARD)); return path
def path2(x, y, phi):

    phi = M(phi); path = []; (rho, t1) = R(x + umath.sin(phi), y - 1 - umath.cos(phi))
    if rho * rho >= 4: u = umath.sqrt(rho * rho - 4); t = M(t1 + umath.atan2(2, u)); v = M(t - phi); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, STRAIGHT, FORWARD)); path.append(create_path_element(v, RIGHT, FORWARD))
    return path
def path3(x, y, phi):

    path = []; xi = x - umath.sin(phi); eta = y - 1 + umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho <= 4: A = umath.acos(rho / 4); t = M(theta + umath.pi / 2 + A); u = M(umath.pi - 2 * A); v = M(phi - t - u); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, RIGHT, BACKWARD)); path.append(create_path_element(v, LEFT, FORWARD))
    return path
def path4(x, y, phi):

    path = []; xi = x - umath.sin(phi); eta = y - 1 + umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho <= 4: A = umath.acos(rho / 4); t = M(theta + umath.pi / 2 + A); u = M(umath.pi - 2 * A); v = M(t + u - phi); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, RIGHT, BACKWARD)); path.append(create_path_element(v, LEFT, BACKWARD))
    return path
def path5(x, y, phi):

    path = []; xi = x - umath.sin(phi); eta = y - 1 + umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho <= 4: u = umath.acos(1 - rho ** 2 / 8); A = umath.asin(2 * umath.sin(u) / rho); t = M(theta + umath.pi / 2 - A); v = M(t - u - phi); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, RIGHT, FORWARD)); path.append(create_path_element(v, LEFT, BACKWARD))
    return path
def path6(x, y, phi):

    path = []; xi = x + umath.sin(phi); eta = y - 1 - umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho <= 4:
        if rho <= 2: A = umath.acos((rho + 2) / 4); t = M(theta + umath.pi / 2 + A); u = M(A); v = M(phi - t + 2 * u)
        else: A = umath.acos((rho - 2) / 4); t = M(theta + umath.pi / 2 - A); u = M(umath.pi - A); v = M(phi - t + 2 * u)
        path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, RIGHT, FORWARD)); path.append(create_path_element(u, LEFT, BACKWARD)); path.append(create_path_element(v, RIGHT, BACKWARD))
    return path
def path7(x, y, phi):

    path = []; xi = x + umath.sin(phi); eta = y - 1 - umath.cos(phi); (rho, theta) = R(xi, eta); u1 = (20 - rho * rho) / 16
    if rho <= 6 and 0 <= u1 <= 1: u = umath.acos(u1); A = umath.asin(2 * umath.sin(u) / rho); t = M(theta + umath.pi / 2 + A); v = M(t - phi); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, RIGHT, BACKWARD)); path.append(create_path_element(u, LEFT, BACKWARD)); path.append(create_path_element(v, RIGHT, FORWARD))
    return path
def path8(x, y, phi):

    path = []; xi = x - umath.sin(phi); eta = y - 1 + umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho >= 2: u = umath.sqrt(rho * rho - 4) - 2; A = umath.atan2(2, u + 2); t = M(theta + umath.pi / 2 + A); v = M(t - phi + umath.pi / 2); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(umath.pi / 2, RIGHT, BACKWARD)); path.append(create_path_element(u, STRAIGHT, BACKWARD)); path.append(create_path_element(v, LEFT, BACKWARD))
    return path
def path9(x, y, phi):

    path = []; xi = x - umath.sin(phi); eta = y - 1 + umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho >= 2: u = umath.sqrt(rho * rho - 4) - 2; A = umath.atan2(u + 2, 2); t = M(theta + umath.pi / 2 - A); v = M(t - phi - umath.pi / 2); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, STRAIGHT, FORWARD)); path.append(create_path_element(umath.pi / 2, RIGHT, FORWARD)); path.append(create_path_element(v, LEFT, BACKWARD))
    return path
def path10(x, y, phi):

    path = []; xi = x + umath.sin(phi); eta = y - 1 - umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho >= 2: t = M(theta + umath.pi / 2); u = rho - 2; v = M(phi - t - umath.pi / 2); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(umath.pi / 2, RIGHT, BACKWARD)); path.append(create_path_element(u, STRAIGHT, BACKWARD)); path.append(create_path_element(v, RIGHT, BACKWARD))
    return path
def path11(x, y, phi):

    path = []; xi = x + umath.sin(phi); eta = y - 1 - umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho >= 2: t = M(theta); u = rho - 2; v = M(phi - t - umath.pi / 2); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(u, STRAIGHT, FORWARD)); path.append(create_path_element(umath.pi / 2, LEFT, FORWARD)); path.append(create_path_element(v, RIGHT, BACKWARD))
    return path
def path12(x, y, phi):

    path = []; xi = x + umath.sin(phi); eta = y - 1 - umath.cos(phi); (rho, theta) = R(xi, eta)
    if rho >= 4: u = umath.sqrt(rho * rho - 4) - 4; A = umath.atan2(2, u + 4); t = M(theta + umath.pi / 2 + A); v = M(t - phi); path.append(create_path_element(t, LEFT, FORWARD)); path.append(create_path_element(umath.pi / 2, RIGHT, BACKWARD)); path.append(create_path_element(u, STRAIGHT, BACKWARD)); path.append(create_path_element(umath.pi / 2, LEFT, BACKWARD)); path.append(create_path_element(v, RIGHT, FORWARD))
    return path
def reverse_gear(element):
 element['gear'] *= -1; return element
def reverse_steering(element):
 element['steering'] *= -1; return element
def reflect(path):

    new_path = []
    for element in path: new_path.append(reverse_steering(element.copy()))
    return new_path
def timeflip(path):

    new_path = []
    for element in path: new_path.append(reverse_gear(element.copy()))
    return new_path
def get_best_path(start_point, end_point):

    path_fns = [path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12]; (x, y, theta) = change_of_basis(start_point, end_point); best = None; best_len = 0.0
    for get_path in path_fns:
        variants = (get_path(x, y, theta), timeflip(get_path(-x, y, -theta)), reflect(get_path(x, -y, -theta)), reflect(timeflip(get_path(-x, -y, theta))))
        for v in variants:
            if not v: continue
            d = 0.0
            for p in v: d += p['distance']
            if best is None or d < best_len: best_len = d; best = v
        gc.collect()
    return best
LOOP_MS = 5; KI_REF_MS = 2.34
class Settings:

    def __init__(self):
     self.velocity = v; self.profile = 'TRAP'; self.accel = 80.0; self.gains = {'kp_pos': KP_POSITION, 'ki_pos': KI_POSITION, 'kp_st': KP_STEERING, 'ki_st': KI_STEERING}; self.legs = None; self.legs_total = 0
cfg = Settings()
def parse_path(arg):

    poses = []
    for chunk in arg.split(';'):
        chunk = chunk.strip()
        if not chunk: continue
        parts = chunk.split(',')
        if len(parts) != 3: return None
        try: poses.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except: return None
    return poses
def set_gains(arg):

    parts = arg.split(',')
    if len(parts) != 4: return False
    try: cfg.gains['kp_pos'] = float(parts[0]); cfg.gains['ki_pos'] = float(parts[1]); cfg.gains['kp_st'] = float(parts[2]); cfg.gains['ki_st'] = float(parts[3])
    except: return False
    return True
def set_velocity(arg):

    try: val = float(arg)
    except: return False
    if val <= 0: return False
    cfg.velocity = val; return True
def set_profile(arg):

    name = arg.strip().upper()
    if name != 'CONST' and name != 'TRAP': return False
    cfg.profile = name; return True
def set_accel(arg):

    try: val = float(arg)
    except: return False
    if val < 0: return False
    cfg.accel = val; return True
V_MIN_REF = 2.0
class ConstantProfile:

    def __init__(self, distance, v_max, h_ms):
    
        step = h_ms * v_max / 1000.0; n = int(distance / step)
        if n * step < distance: n += 1
        if n < 1: n = 1
        self.n = n; self.ds = distance / n; self.ticks = 0
    def next(self):
     self.ticks += 1; return (self.ds, self.ticks >= self.n)
class TrapezoidProfile:

    def __init__(self, distance, v0, v1, v_max, accel, h_ms):
     self.distance = distance; self.v1 = v1; self.v_max = v_max; self.accel = accel; self.h_ms = h_ms; self.v = v0; self.s = 0.0
    def next(self):
    
        remaining = self.distance - self.s; v_up = self.v + self.accel * self.h_ms / 1000.0; v_brake = umath.sqrt(self.v1 * self.v1 + 2.0 * self.accel * remaining); v = min(self.v_max, v_up, v_brake)
        if v < V_MIN_REF: v = V_MIN_REF
        self.v = v; ds = v * self.h_ms / 1000.0; last = ds >= remaining
        if last: ds = remaining
        self.s += ds; return (ds, last)
def make_profile(kind, distance, gear, prev_gear, next_gear, v_max, accel, h_ms):

    if kind == 'TRAP' and accel > 0: v0 = v_max if prev_gear == gear else 0.0; v1 = v_max if next_gear == gear else 0.0; return TrapezoidProfile(distance, v0, v1, v_max, accel, h_ms)
    return ConstantProfile(distance, v_max, h_ms)
class Reference:

    def __init__(self):
     self.heading = 0.0; self.x = 0.0; self.y = 0.0; self.position = 0.0; self.done = True; self._heading0 = 0.0; self._distance = 0.0; self._steer_dir = 0; self._gear = 1; self._profile = None
    def start_segment(self, distance, steer_dir, gear, profile):
     self._heading0 = self.heading; self._distance = distance; self._steer_dir = steer_dir; self._gear = gear; self._profile = profile; self.position = 0.0; self.done = False
    def advance(self):
    
        if self.done: return
        gear = self._gear; (ds, last) = self._profile.next()
        if not last: self.position += gear * ds; self.heading += gear * self._steer_dir * ds / r_turn_min
        else: self.position = gear * self._distance; self.heading = self._heading0 + gear * self._steer_dir * self._distance / r_turn_min; self.done = True
        th = -self.heading; self.x += gear * ds * umath.cos(th); self.y += gear * ds * umath.sin(th)
class Odometry:

    def __init__(self):
     self.x = 0.0; self.y = 0.0; self._pos_prev = 0.0
    def reset_segment(self):
     self._pos_prev = 0.0
    def update(self, position_vehicle, heading_deg):
     ds = position_vehicle - self._pos_prev; self._pos_prev = position_vehicle; heading_rad = -umath.radians(heading_deg); self.x += ds * umath.cos(heading_rad); self.y += ds * umath.sin(heading_rad); return ds
    def tracking_errors(self, ref):
     th_r = -ref.heading; c_r = umath.cos(th_r); s_r = umath.sin(th_r); dx = self.x - ref.x; dy = self.y - ref.y; e_s = c_r * dx + s_r * dy; e_y = -s_r * dx + c_r * dy; return (e_s, e_y)
def make_frame(leg_start):

    th0 = umath.radians(leg_start[2]); c = umath.cos(th0); s = umath.sin(th0); x0 = leg_start[0]; y0 = leg_start[1]
    def to_global(xl, yl):
     return (x0 + xl * c - yl * s, y0 + xl * s + yl * c)
    return to_global
ANGLE_TO_CM = 0.045; hub = TechnicHub(); steering = Motor(Port.D, Direction.CLOCKWISE); front = Motor(Port.B, Direction.CLOCKWISE); rear = Motor(Port.A, Direction.CLOCKWISE); audi = Car(steering, [front, rear]); run_watch = StopWatch()
def setup():
 hub.light.on(Color.RED); hub.imu.settings(1.5, 250)
def stop_car():
 audi.drive_power(0); audi.steer(0)
def reset_sensors():
 audi.drive_power(0); audi.steer(0); hub.imu.reset_heading(0); front.reset_angle(0); rear.reset_angle(0)
def reset_encoders():
 front.reset_angle(0); rear.reset_angle(0)
def read_position():
 return 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM
def read_heading():
 return hub.imu.heading()
def motors_stalled():
 return rear.stalled() or front.stalled()
def battery():

    try: return (hub.battery.voltage(), hub.battery.current())
    except: return (0, 0)
_poller = poll(); _poller.register(stdin); _inbuf = ''
def read_command():

    global _inbuf
    while _poller.poll(0):
        ch = stdin.buffer.read(1)
        if not ch: break
        b = ch[0]
        if b == 10 or b == 13:
            if not _inbuf: continue
            line = _inbuf; _inbuf = ''; return line.strip()
        else: _inbuf += chr(b)
    return None
def send_battery():
 (mv, ma) = battery(); print('BAT {},{}'.format(mv, ma))
def send_telemetry(t, pos_ref, pos_veh, pos_err, pos_cmd, st_ref_deg, st_veh, st_err, st_cmd, x, y, pP, pI, sP, sI, ey, ss):
 print('T {:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.2f},{:.3f}'.format(t, pos_ref, pos_veh, pos_err, pos_cmd, st_ref_deg, st_veh, st_err, st_cmd, x, y, pP, pI, sP, sI, ey, ss))
MIN_SEG_DISTANCE = 1.0
def plan_leg(start_pose, end_pose):

    (sp, ep) = normalize_start_and_end_point(start_pose, end_pose, r_turn_min); best = get_best_path(sp, ep)
    if best is None: return None
    return denormalize_distance_in_path(best, r_turn_min)
def build_legs(PATH):

    legs = []; seg_total = 0
    for i in range(len(PATH) - 1):
        leg = plan_leg(PATH[i], PATH[i + 1])
        if leg is None: return (None, 0)
        leg = [seg for seg in leg if seg['distance'] >= MIN_SEG_DISTANCE]; legs.append(leg); seg_total += len(leg); gc.collect()
    return (legs, seg_total)
def stream_plan(path, leg_start, step=8.0, batch=14):

    to_global = make_frame(leg_start); xl = 0.0; yl = 0.0; thl = 0.0; parts = ['{:.1f},{:.1f}'.format(leg_start[0], leg_start[1])]
    for seg in path:
        distance = seg['distance']; steer_dir = seg['steering']; gear = seg['gear']; travelled = 0.0
        while travelled < distance:
            ds = step
            if distance - travelled < step: ds = distance - travelled
            travelled += ds; thl -= gear * steer_dir * ds / r_turn_min; xl += gear * ds * umath.cos(thl); yl += gear * ds * umath.sin(thl); (gx, gy) = to_global(xl, yl); parts.append('{:.1f},{:.1f}'.format(gx, gy))
            if len(parts) >= batch: print('PLAN ' + ';'.join(parts)); parts = []
    if parts: print('PLAN ' + ';'.join(parts))
def send_plan(legs, PATH):

    for i in range(len(legs)): gc.collect(); stream_plan(legs[i], PATH[i])
def saturate(value, limit):
 return int(min(max(value, -limit), limit))
def integrate(integral, p, delta, limit):

    new = integral + delta; u = p + new
    if u > limit and delta > 0 or (u < -limit and delta < 0): new = integral
    if new > limit: new = limit
    elif new < -limit: new = -limit
    return new
class PI:

    def __init__(self, kp_key, ki_key, limit, h=LOOP_MS):
     self.kp_key = kp_key; self.ki_key = ki_key; self.limit = limit; self.ki_scale = h / KI_REF_MS; self.integral = 0.0
    def reset(self):
     self.integral = 0.0
    def update(self, error):
     p = cfg.gains[self.kp_key] * error; self.integral = integrate(self.integral, p, cfg.gains[self.ki_key] * self.ki_scale * error, self.limit); return (saturate(p + self.integral, self.limit), p, self.integral)
def make_position_controller(h=LOOP_MS):
 return PI('kp_pos', 'ki_pos', max_drive_power, h)
class SteeringPI:

    def __init__(self, h=LOOP_MS):
     self.pi = PI('kp_st', 'ki_st', max_angle_power, h); self.s = 0.0
    def reset(self):
     self.pi.reset()
    def update(self, gear, steering_error):
     (command, p, i) = self.pi.update(steering_error); return (gear * command, p, i)
def make_steering_controller(h=LOOP_MS):
 return SteeringPI(h)
def controlDelVehiculo(path, leg_start, seg_offset, seg_total, h=LOOP_MS):

    elapsed_time_max = 30; elapsed_time = 0; reset_sensors(); ref = Reference(); odo = Odometry(); pos_ctrl = make_position_controller(h); steer_ctrl = make_steering_controller(h); to_global = make_frame(leg_start); loop_ticks = 0; overruns = 0; loop_t0 = run_watch.time(); next_tick = loop_t0
    for i in range(len(path)):
        print('SEG {} {}'.format(seg_offset + i + 1, seg_total)); distance = path[i]['distance']; steer_dir = path[i]['steering']; gear = path[i]['gear']; prev_gear = path[i - 1]['gear'] if i > 0 else None; nxt = path[i + 1] if i + 1 < len(path) else None; next_gear = nxt['gear'] if nxt is not None else None; profile = make_profile(cfg.profile, distance, gear, prev_gear, next_gear, cfg.velocity, cfg.accel, h); ref.start_segment(distance, steer_dir, gear, profile); reset_encoders(); odo.reset_segment(); position_vehicle = 0.0; pos_ctrl.reset(); steer_ctrl.reset()
        while abs(position_vehicle) <= distance and elapsed_time / 1000.0 < elapsed_time_max:
            if read_command() == 'STOP': stop_car(); return 'stop'
            if motors_stalled(): stop_car(); return 'stall'
            ref.advance(); position_vehicle = read_position(); steering_vehicle = read_heading(); position_error = ref.position - position_vehicle; steering_error = umath.degrees(ref.heading) - steering_vehicle; odo.update(position_vehicle, steering_vehicle); (e_s, e_y) = odo.tracking_errors(ref); (position_command, position_P, position_I) = pos_ctrl.update(position_error); (steering_command, steering_P, steering_I) = steer_ctrl.update(gear, steering_error); audi.steer(steering_command); audi.drive_power(position_command); next_tick += h; rem = next_tick - run_watch.time()
            if rem > 0: wait(rem)
            elif rem < 0:
                overruns += 1
                if rem < -4 * h: next_tick = run_watch.time()
            elapsed_time += h; loop_ticks += 1
            if elapsed_time % 100 == 0: (x_global, y_global) = to_global(odo.x, odo.y); send_telemetry(run_watch.time() / 1000.0, ref.position, position_vehicle, position_error, position_command, umath.degrees(ref.heading), steering_vehicle, steering_error, steering_command, x_global, y_global, position_P, position_I, gear * steering_P, gear * steering_I, e_y, steer_ctrl.s)
        if elapsed_time / 1000.0 >= elapsed_time_max: stop_car(); return 'timeout'
    stop_car()
    if loop_ticks > 0: print('LOOP {:.2f} {}'.format((run_watch.time() - loop_t0) / loop_ticks, overruns))
    return 'ok'
def run_trajectory(PATH):

    run_watch.reset(); hub.light.on(Color.ORANGE)
    if cfg.legs is None:
        (cfg.legs, cfg.legs_total) = build_legs(PATH)
        if cfg.legs is None: print('ERR sin solucion'); print('END error'); hub.light.on(Color.RED); return
        send_plan(cfg.legs, PATH)
    legs = cfg.legs; seg_total = cfg.legs_total; gc.collect(); seg_offset = 0
    for i in range(len(legs)):
        status = controlDelVehiculo(legs[i], PATH[i], seg_offset, seg_total); seg_offset += len(legs[i])
        if status != 'ok': print('END ' + status); hub.light.on(Color.RED); return
    print('END ok'); hub.light.on(Color.GREEN)
SETTERS = {'PI': set_gains, 'PROF': set_profile, 'ACC': set_accel, 'VEL': set_velocity}
def main():

    setup(); PATH = None; print('RDY'); print('VER 17 RS+PI'); send_battery()
    while True:
        line = read_command()
        if line is None: wait(20); continue
        if line == '': continue
        p = line.split(' ', 1); cmd = p[0]; arg = p[1] if len(p) > 1 else ''
        if cmd == 'PATH':
            poses = parse_path(arg)
            if poses and len(poses) >= 2:
                PATH = poses; (cfg.legs, cfg.legs_total) = build_legs(PATH)
                if cfg.legs is None: print('ERR sin solucion')
                else: print('ACK PATH {}'.format(len(PATH))); send_plan(cfg.legs, PATH)
            else: PATH = None; cfg.legs = None; print('ERR PATH')
        elif cmd in SETTERS: print(('ACK ' if SETTERS[cmd](arg) else 'ERR ') + line)
        elif cmd == 'START':
            if not PATH: print('ERR sin PATH')
            else: print('ACK START'); run_trajectory(PATH); send_battery(); print('RDY')
        elif cmd == 'STOP': stop_car(); print('ACK STOP')
        elif cmd == 'RESET': reset_sensors(); send_battery(); print('ACK RESET')
        elif cmd == 'BAT': send_battery()
        elif cmd == 'PING': print('RDY')
        else: print('ERR ' + line)
if __name__ == '__main__': main()
