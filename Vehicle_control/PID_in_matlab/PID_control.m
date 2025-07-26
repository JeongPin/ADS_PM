% 경로 불러오기
data = readmatrix('gnss_log_1m_2.csv');
x_ref = data(:,2);
y_ref = data(:,3);

% 초기값
x = x_ref(1); y = y_ref(1); theta = 0; v = 0;
dt = 0.05;
L = 2.5;  % 축간 거리

% 초기화
int_err_speed = 0;
prev_err_speed = 0;
int_err_steer = 0;
prev_err_steer = 0;


% PID 파라미터
Kp_speed = 0.6; Ki_speed = 0.1; Kd_speed = 0.05;
Kp_steer = 1.2; Ki_steer = 0.0; Kd_steer = 0.2;

% 로그 저장용
x_log = []; y_log = []; theta_log = [];

% 시뮬레이션 루프
for i = 1:2000  % 예시 루프
    % 목표 속도
    target_speed = 8;

    % 현재 위치에서 가장 가까운 reference point
    dist = hypot(x_ref - x, y_ref - y);
    [~, idx] = min(dist);
    idx = min(idx+5, length(x_ref));
    tx = x_ref(idx); ty = y_ref(idx);

    % 속도 PID
    speed_error = target_speed - v;
    int_err_speed = int_err_speed + speed_error * dt;
    der_err_speed = (speed_error - prev_err_speed) / dt;
    throttle = Kp_speed * speed_error + Ki_speed * int_err_speed + Kd_speed * der_err_speed;
    prev_err_speed = speed_error;

    % 조향 PID
    target_yaw = atan2(ty - y, tx - x);
    heading_error = wrapToPi(target_yaw - theta);
    int_err_steer = int_err_steer + heading_error * dt;
    der_err_steer = (heading_error - prev_err_steer) / dt;
    steer = Kp_steer * heading_error + Ki_steer * int_err_steer + Kd_steer * der_err_steer;
    prev_err_steer = heading_error;

    % saturation
    throttle = min(max(throttle, 0), 1);
    steer = min(max(steer, -pi/4), pi/4);

    % saturation
    throttle = min(max(throttle, 0), 1);
    steer = min(max(steer, -pi/4), pi/4);

    % 차량 모델 업데이트
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + (v / L) * tan(steer) * dt;
    v = v + throttle * dt;  % 아주 단순한 가속 모델

    % 로그 저장
    x_log(end+1) = x;
    y_log(end+1) = y;
    theta_log(end+1) = theta;
end

% 결과 시각화
plot(x_ref, y_ref, 'b--'); hold on;
plot(x_log, y_log, 'r'); legend('Reference', 'Actual');
axis equal;