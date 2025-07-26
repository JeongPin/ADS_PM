import carla
import csv
import math
import time

# ───── PID 클래스 정의 ─────
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def run(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# ───── Waypoints 불러오기 ─────
def load_waypoints(csv_path):
    waypoints = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            x, y, z = map(float, row[1:4])
            waypoints.append(carla.Location(x, y, z))
    return waypoints

# ───── 거리 계산 함수 ─────
def distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)

# ───── 가장 가까운 waypoint 찾기 ─────
def find_closest_index(current_location, waypoints):
    min_dist = float('inf')
    closest_idx = 0
    for i, wp in enumerate(waypoints):
        d = distance(current_location, wp)
        if d < min_dist:
            min_dist = d
            closest_idx = i
    return closest_idx

# ───── 앞차 감지 함수 ─────
def detect_lead_vehicle(ego_vehicle, other_vehicles, max_distance=8.0, fov_deg=45.0):
    ego_tf = ego_vehicle.get_transform()
    ego_loc = ego_tf.location
    ego_yaw = math.radians(ego_tf.rotation.yaw)
    closest_dist = float('inf')
    for other in other_vehicles:
        other_loc = other.get_transform().location
        dx = other_loc.x - ego_loc.x
        dy = other_loc.y - ego_loc.y
        dist = math.hypot(dx, dy)
        # 전방 FOV 체크
        heading_to_other = math.atan2(dy, dx)
        angle_diff = (heading_to_other - ego_yaw + math.pi) % (2*math.pi) - math.pi
        if abs(angle_diff) <= math.radians(fov_deg) and dist < max_distance:
            if dist < closest_dist:
                closest_dist = dist
    return closest_dist < float('inf'), closest_dist

# ───── CARLA 초기화 및 트럭 스폰 ─────
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

# ───── 이전에 스폰된 트럭 전부 제거 ─────
for old_truck in world.get_actors().filter('vehicle.carlamotors.carlacola'):
    old_truck.destroy()


bp_lib = world.get_blueprint_library()
truck_bp = bp_lib.filter('vehicle.carlamotors.carlacola')[0]

# 차량 1 소환 (좌표/방향은 임의 설정)
spawn_1 = carla.Transform(carla.Location(x=-22, y=-30, z=2.9),
                          carla.Rotation(yaw=90.0))
vehicle1 = world.try_spawn_actor(truck_bp, spawn_1)

# 차량 2 소환
spawn_2 = carla.Transform(carla.Location(x=-45, y=-24.9, z=2.9),
                          carla.Rotation(yaw=90.0))
vehicle2 = world.try_spawn_actor(truck_bp, spawn_2)

vehicles = [vehicle1, vehicle2]
if any(v is None for v in vehicles):
    raise RuntimeError("❌ 차량 소환에 실패했습니다. 스폰 좌표를 확인하세요.")

# ───── Waypoints 불러오기 ─────
waypoints1 = load_waypoints('gnss_log_1m.csv')
waypoints2 = load_waypoints('gnss_log_1m_2.csv')
waypoints_list = [waypoints1, waypoints2]

# ───── PID 컨트롤러 설정 ─────
pid_speed_list = [
    PIDController(Kp=0.6, Ki=0.1, Kd=0.05),
    PIDController(Kp=0.6, Ki=0.1, Kd=0.05)
]
pid_steer_list = [
    PIDController(Kp=1.2, Ki=0.0, Kd=0.2),
    PIDController(Kp=1.2, Ki=0.0, Kd=0.2)
]

# ───── 중간 정지 지점 설정 ─────
stop_location_1 = carla.Location(x=-169, y=5, z=2.8)
stop_location_2 = carla.Location(x=-176.5, y=5, z=2.8)
stop_locations = [stop_location_1, stop_location_2]
stop_threshold = 5.0
has_stopped = [False, False]

# ───── 최종 정지 플래그 ─────
break_flags = [False, False]

# ───── 로그 파일 초기화 ─────
for idx in range(2):
    with open(f'actual_path_{idx+1}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'x', 'y', 'z'])

# ───── 목표 속도 설정 ─────
TARGET_SPEED = 8.0  # m/s

# ───── 제어 루프 시작 ─────

current_wp_index = [0, 0] # 차량별 waypoint 진행 상태 저장
prev_times = [time.time(), time.time()]
# ───── 정지 타이머 초기화 ─────
stop_timer = [None, None]
stop_duration = [7.0, 10.0]  # 초

for i in range(2):
    with open(f'steer_log_{i+1}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'loc_x', 'loc_y', 'wp_x', 'wp_y', 'speed', 'heading_error_deg', 'steer'])


while True:
    world.tick()
    current_time = time.time()
    # dt = current_time - prev_times[i]
    # prev_times[i] = current_time

    # 모든 차량이 경로 완료했으면 루프 종료
    if all(break_flags):
        print("모든 차량 경로 완료!")
        break

    for i, vehicle in enumerate(vehicles):
        if break_flags[i]:
            continue
        dt = current_time - prev_times[i]
        prev_times[i] = current_time

        # --- 상태 읽기 ---
        tf = vehicle.get_transform()
        loc = tf.location
        yaw = math.radians(tf.rotation.yaw)
        vel = vehicle.get_velocity()
        speed = math.hypot(vel.x, vel.y)

        # --- 현재 목표 waypoint ---
        wps = waypoints_list[i]
        new_idx = find_closest_index(loc, wps)
        if new_idx > current_wp_index[i]:
            current_wp_index[i] = new_idx  #way point 뒤로 가는 것 방지
        target_wp = wps[min(current_wp_index[i] + 5, len(wps) - 1)]

        # --- 중간 정지 처리 ---

        if not has_stopped[i] and distance(loc, stop_locations[i]) < stop_threshold:
            if stop_timer[i] is None:
                print(f"🛑 차량 {i + 1}: 중간 정지 시작")
                stop_timer[i] = current_time  # 정지 시작 시간 기록

            # 정지 시간 아직 경과 안됨
            if current_time - stop_timer[i] < stop_duration[i]:
                ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
                vehicle.apply_control(ctrl)
                continue
            else:
                print(f"▶️ 차량 {i + 1}: 정지 종료, 주행 재개")
                has_stopped[i] = True  # 정지 완료 플래그
        # --- 앞차 감지 & 정지 ---
        others = vehicles[:i] + vehicles[i+1:]
        detected, dist_ahead = detect_lead_vehicle(vehicle, others,
                                                   max_distance=10.0,
                                                   fov_deg=30.0)
        if detected:
            print(f"⛔ 차량 {i+1}: 앞차 감지 ({dist_ahead:.1f}m) → 정지")
            ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
            vehicle.apply_control(ctrl)
            continue


        # --- 커브 감속 판단 ---
        dx = target_wp.x - loc.x
        dy = target_wp.y - loc.y
        heading_error = math.atan2(dy, dx) - yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        if abs(math.degrees(heading_error)) > 25:
            target_speed = 2.0
            print(f"[차량{i + 1}] heading_error: {math.degrees(heading_error):.1f} deg → target_speed: {target_speed}")
        else:
            target_speed = 8.0



        # --- PID 제어 ---
        # 종방향 (속도)
        speed_error = TARGET_SPEED - speed
        throttle = pid_speed_list[i].run(speed_error, dt)
        throttle = max(0.0, min(1.0, throttle))

        # 횡방향 (조향)
        dx = target_wp.x - loc.x
        dy = target_wp.y - loc.y
        heading_error = math.atan2(dy, dx) - yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        steer = pid_steer_list[i].run(heading_error, dt)
        steer = max(-1.0, min(1.0, steer))
        ctrl = carla.VehicleControl(throttle=throttle, steer=steer, brake=0.0)

        with open(f'steer_log_{i + 1}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time,
                loc.x, loc.y,
                target_wp.x, target_wp.y,
                speed,
                math.degrees(heading_error),  # deg로 변환
                steer
            ])


        vehicle.apply_control(ctrl)

        # --- 로그 기록 ---
        with open(f'actual_path_{i+1}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, loc.x, loc.y, loc.z])

        # --- 디버그 시각화 ---
        world.debug.draw_point(target_wp, size=0.2,
                               color=carla.Color(0, 255, 0), life_time=0.1)
        world.debug.draw_point(loc, size=0.2,
                               color=carla.Color(255, 255, 0), life_time=0.1)

        # 차량 i, 루프 내 PID 제어 직전에 삽입
        final_loc = waypoints_list[i][-1]
        dist_final = distance(loc, final_loc)

        # 최대 허용 감속도 (실험적으로 조정)
        max_decel = 3.0  # m/s^2

        # 제동 진입 거리
        brake_zone = speed ** 2 / (2 * max_decel + 1e-6)

        if dist_final < brake_zone:
            # 0→1 사이로 브레이크 비율 계산
            brake_ratio = min(1.0, (brake_zone - dist_final) / brake_zone)
            ctrl = carla.VehicleControl(throttle=0.0,
                                        steer=steer,  # PID로 계산된 steer 유지
                                        brake=brake_ratio)
            vehicle.apply_control(ctrl)
            # 로그나 디버그 컬러로 브레이크 상태 시각화 가능
            continue


        # --- 최종 정지 (경로 완료) ---
        if current_wp_index[i] >= len(wps) - 2:
            print(f"✅ 차량 {i+1} 경로 완료")
            # 완전히 정지
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            break_flags[i] = True

