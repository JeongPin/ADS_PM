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
        self.prev_error = 0
        self.integral = 0

    def run(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
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

# ───── CARLA 초기화 및 트럭 스폰 ─────
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

for old_truck in world.get_actors().filter('vehicle.carlamotors.carlacola'):
    old_truck.destroy()


bp_lib = world.get_blueprint_library()
truck_bp = bp_lib.filter('vehicle.carlamotors.carlacola')[0]  # 트럭 블루프린트

spawn_location = carla.Location(x=-45, y=-24.9, z=2.9)
spawn_rotation = carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0)
spawn_transform = carla.Transform(spawn_location, spawn_rotation)

truck = world.try_spawn_actor(truck_bp, spawn_transform)

if truck is None:
    raise RuntimeError("❌ 트럭 스폰 실패: 다른 객체가 이미 그 자리에 있을 수 있습니다.")

vehicle = truck  # 제어 대상 차량

# ───── Waypoints 불러오기 ─────
waypoints = load_waypoints('gnss_log_1m_2.csv')

# ───── PID 컨트롤러 설정 ─────
pid_speed = PIDController(Kp=0.6, Ki=0.1, Kd=0.05)
pid_steer = PIDController(Kp=1.2, Ki=0.0, Kd=0.2)

# ───── 정지 지점 설정 ─────
stop_location = carla.Location(x=-176.5, y=5, z=2.8)
stop_threshold = 3.0  # m 이내로 접근하면 정지
has_stopped = False

# ───── 목표 속도 설정 ─────
TARGET_SPEED = 8.0  # m/s (약 28.8km/h)

# ───── 제어 루프 시작 ─────
prev_time = time.time()
# with open('actual_path.csv', 'w', newline='') as f:
#     writer = csv.writer(f)
#     writer.writerow(['time', 'x', 'y', 'z'])


while True:
    world.tick()
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    transform = vehicle.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)

    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2)

    closest_idx = find_closest_index(location, waypoints)
    target_wp = waypoints[min(closest_idx + 5, len(waypoints) - 1)]

    # ───── 제어 계산 ─────
    speed_error = TARGET_SPEED - speed
    throttle = pid_speed.run(speed_error, dt)
    throttle = max(0.0, min(1.0, throttle))

    dx = target_wp.x - location.x
    dy = target_wp.y - location.y
    heading_error = math.atan2(dy, dx) - yaw

    # pergola 앞 정지 처리
    if not has_stopped and distance(location, stop_location) < stop_threshold:
        print("🛑 정지 지점 도달, 일시 정차 중...")
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        vehicle.apply_control(control)
        time.sleep(10.0)  # 3초 정지
        has_stopped = True



    # 각도 wrap-around 보정 (-pi ~ pi)
    heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

    steer = pid_steer.run(heading_error, dt)
    steer = max(-1.0, min(1.0, steer))

    control = carla.VehicleControl()
    control.throttle = throttle
    control.steer = steer
    control.brake = 0.0
    vehicle.apply_control(control)
    # ───── 위치 로그 저장 ─────
    # with open('actual_path.csv', 'a', newline='') as f:
    #     writer = csv.writer(f)
    #     writer.writerow([current_time, location.x, location.y, location.z])
    # ───── 시각화 ─────
    world.debug.draw_point(target_wp, size=0.2, color=carla.Color(0, 255, 0), life_time=0.2)
    world.debug.draw_point(location, size=0.2, color=carla.Color(255, 255, 0), life_time=0.2)

    # 종료 조건
    if closest_idx >= len(waypoints) - 2:
        print("✅ 경로 완료")
        break