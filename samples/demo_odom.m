%% demo_odom —— ChassisClient 示例：低速直行、采集里程计并绘图
% 前置：chassis_host 已在运行（本机或小车树莓派），本脚本与 ChassisClient.m
% 同路径。真车演示请确认场地空旷、随时可以 c.stop()。
% 命令与遥测说明见 chassis-host/docs/protocol-tcp.md。

clear; clc;

HOST = "172.27.206.9";   % 改成小车 IP，如 "192.168.1.10"
PORT = 9000;

c = ChassisClient(HOST, PORT);
c.connect();
fprintf('已连接 %s:%d\n', char(c.Host), c.Port);

% 订阅遥测并等待链路 READY（宿主自动建链 + 自动使能，见 chassis-host/docs/protocol-serial.md）
c.cmd("watch on");
c.startTelemetry(20);
assert(c.waitReady(10), ...
    '链路 10s 内未 READY：检查板子与串口，宿主侧可用 con/stat 命令查状态');

% 低速直行 3s 后刹车；宿主以 10Hz 重发该指令，客户端只需发一次
c.clearTelemetry();
c.drive(0.15, 0, 0);
pause(3);
c.stop();

% odom 以 --odom-hz（默认 10Hz）推送，留 0.5s 收尾最后一拍
pause(0.5);
o = c.getOdom();
assert(numel(o.t) > 0, '未收到 odom 遥测：确认已 watch on 且遥测定时器在跑');
fprintf('采集到 %d 拍 odom，末点 x=%.3f y=%.3f yaw=%.3f\n', ...
    numel(o.t), o.x(end), o.y(end), o.yaw(end));

figure('Name', 'odom 轨迹');
plot(o.x, o.y, '-o', 'MarkerSize', 3);
axis equal; grid on;
xlabel('x / m'); ylabel('y / m'); title('低速直行里程计轨迹');

figure('Name', '速度曲线');
plot(o.t, o.vx, o.t, o.wz);
grid on; legend('vx / (m/s)', 'wz / (rad/s)');
xlabel('t / s');

c.disconnect();
