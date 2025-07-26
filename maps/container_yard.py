import csv
import carla
import time
# 클라이언트 연결
client = carla.Client('localhost', 2000)
client.set_timeout(30.0)
world = client.load_world('Town07_OPT')

# Toggle all buildings off
world.unload_map_layer(carla.MapLayer.Buildings)
world.unload_map_layer(carla.MapLayer.Foliage)
# 야간 + 흐림 + 안개 환경
weather = carla.WeatherParameters(
    sun_altitude_angle=10.0,   # 낮
    cloudiness=10.0,
    fog_density=10.0,
    wetness=60.0
)
world.set_weather(weather)

# 블루프린트 & 디버그 툴
blueprint_library = world.get_blueprint_library()
container_bp = blueprint_library.find('static.prop.container')
debug = world.debug
container_actors = []

# 기준 좌표 (Town07 평지 기준)
base_x, base_y, z = -165, 35, 0.5

# 컨테이너 위치 오프셋 (3열 * 16칸, 사각형 형태)
box_offsets = [
    (i * -6, -(j-1) * 4) for i in range(3) for j in range(17)
]

# 컨테이너 생성
for i, (dx, dy) in enumerate(box_offsets):
    loc = carla.Location(x=base_x + dx, y=base_y + dy, z=z)
    rot = carla.Rotation(yaw=90 )
    tf = carla.Transform(loc, rot)

    actor = world.try_spawn_actor(container_bp, tf)
    if actor:
        container_actors.append(actor)
        print(f"[OK] Spawned container {i} at ({loc.x:.1f}, {loc.y:.1f})")
    else:
        print(f"[FAIL] Could not spawn container {i} at ({loc.x:.1f}, {loc.y:.1f})")



# 퍼골라 블루프린트
crane_bp = blueprint_library.find('static.prop.pergola')

# 퍼골라 위치 설정 (야적장 입구, 왼쪽에 고정)
crane_loc = carla.Location(x=base_x - 18, y=base_y - 30, z=0.5)
crane_rot = carla.Rotation(yaw=0)
crane_tf = carla.Transform(crane_loc, crane_rot)


crane_locB = carla.Location(x=base_x+2.5, y=base_y - 30, z=0.5)
crane_rotB = carla.Rotation(yaw=0)
crane_tfB = carla.Transform(crane_locB, crane_rotB)

# 크레인 생성
crane_actor = world.try_spawn_actor(crane_bp, crane_tf)
crane_actorB = world.try_spawn_actor(crane_bp, crane_tfB)
if crane_actor:
    print(f"[OK] Crane placed at ({crane_loc.x:.1f}, {crane_loc.y:.1f})")
else:
    print(f"[FAIL] Failed to spawn crane at ({crane_loc.x:.1f}, {crane_loc.y:.1f})")


# ─── CSV 파일 불러오기 ───
csv_file = 'gnss_log_1m.csv'

with open(csv_file, 'r') as f:
    reader = csv.reader(f)
    next(reader)  # skip header
    for row in reader:
        x, y, z = map(float, row[1:4])

        world.debug.draw_point(
            carla.Location(x=x, y=y, z=z + 0.5),  # 약간 띄워서 보이게
            size=0.2,
            color=carla.Color(r=0, g=255, b=0),  # 초록색 점
            life_time=60.0,  # 60초 동안 보이기
            persistent_lines=False  # True면 계속 보임
        )

print("✅ 맵 위에 waypoint 점 찍기 완료")



# 시점 설정 (위에서 야적장 전체를 바라보도록)
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(
    location=carla.Location(x=base_x + 20, y=base_y - 30, z=50),
    rotation=carla.Rotation(pitch=-90)
))

print(f"\n[INFO] Total {len(container_actors)} containers spawned.")
print("[INFO] Spectator moved. Yard setup in Town07 complete.")
