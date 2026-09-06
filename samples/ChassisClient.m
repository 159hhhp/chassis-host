classdef ChassisClient < handle
    %ChassisClient —— chassis_host 的 MATLAB TCP 客户端
    %
    %连接 chassis-host 的 TCP 文本协议口（一行一命令，应答 ok/err，遥测为
    %JSON 行），下发指令并把遥测解析进缓冲区，供 MATLAB 采集数据、绘图
    %与闭环实验。链路保活（CON 建链/看门狗）与速度指令 10Hz 重发均由
    %宿主完成，本类只说文本协议；命令表见 chassis-host/docs/protocol-tcp.md，
    %链路保活与失效保护见 chassis-host/docs/protocol-serial.md。
    %
    %环境：基础 MATLAB R2020b 及以上，无需任何工具箱。
    %
    %快速上手：
    %   c = ChassisClient("192.168.1.10");
    %   c.connect();
    %   c.cmd("watch on");        % 订阅 JSON 遥测
    %   c.startTelemetry();       % 后台定时器接收遥测
    %   c.waitReady();            % 等宿主建链并自动使能电机
    %   c.drive(0.15, 0, 0);      % 低速前进（宿主 10Hz 重发，发一次即可）
    %   pause(3);
    %   c.stop();                 % 刹车并停止重发
    %   o = c.getOdom();
    %   figure; plot(o.x, o.y, "-o"); axis equal; grid on;
    %   c.disconnect();
    %
    %See also demo_odom.m

    properties (SetAccess = private)
        Host (1,1) string = "127.0.0.1"
        Port (1,1) double = 9000
        IsConnected (1,1) logical = false
        Odom = {}          % cell of struct：t,vx,vy,wz,x,y,yaw（速率 --odom-hz，默认 10Hz）
        Enc = {}           % cell of struct：t,count(1x3),rpm(1x3)（--slow-hz，默认 1Hz）
        Imu = {}           % cell of struct：t,accel_g,gyro_dps,rpy_deg（各 1x3）
        Telem = {}         % cell of struct：t,tgt_rpm,rpm,out_pct（各 1x3）
        Events = {}        % cell of struct：t,type,data（con/serial/board 事件，data 为原始 JSON struct）
        Messages = {}      % cell of struct：t,text（非 JSON 行：欢迎语/ok/err/pong/help，排查用）
        LatestStatus = []  % 最近一拍 status 解码结果：t,uptime,imu_ok,en,mode,con
    end

    properties (Access = private)
        tcp_ = []
        timer_ = []
        rxbuf_ = []      % uint8 行缓冲：已读入但未凑满一行的残包
        ackQueue_ = {}   % cell of char：ok/err/pong 应答行，由 cmd() 取走；
                         % 定时器与 cmd 共用收流，应答必须归队而非被定时器丢弃
        t0_ = []         % connect 时刻（tic 句柄），遥测时间戳以此为 0
    end

    methods
        function obj = ChassisClient(host, port)
            if nargin >= 1, obj.Host = string(host); end
            if nargin >= 2, obj.Port = port; end
        end

        function connect(obj, timeout)
            %建立连接并消费欢迎行与首条 con 事件
            if nargin < 2, timeout = 5; end
            obj.tcp_ = tcpclient(obj.Host, obj.Port, "Timeout", timeout);
            configureTerminator(obj.tcp_, "lf");
            obj.rxbuf_ = [];
            obj.ackQueue_ = {};
            obj.t0_ = tic;
            obj.IsConnected = true;
            tEnd = tic;
            while toc(tEnd) < 1.0
                obj.pumpLines();
                if ~isempty(obj.Events), break; end
                pause(0.02);
            end
        end

        function disconnect(obj)
            %断开连接，断开前 best-effort 补发一次 stop
            %宿主会因速度指令来源断开而自动 CMD_STOP（docs/protocol-tcp.md），
            %补发覆盖本连接不是指令来源的场合
            if ~obj.IsConnected, return; end
            try
                obj.cmd("stop", 0.5);
            catch  % 连接已失效，无需应答
            end
            obj.stopTelemetry();
            obj.IsConnected = false;
            if ~isempty(obj.tcp_) && isvalid(obj.tcp_)
                delete(obj.tcp_);
            end
            obj.tcp_ = [];
        end

        function [isok, ack] = cmd(obj, line, timeout)
            %下发一行指令并阻塞等待应答
            %应答取自 ackQueue_ 的首个 ok/err 行（ping 以 pong 行为应答）；
            %期间到达的 JSON 遥测照常入缓冲。超时返回 isok=false
            if nargin < 3, timeout = 2.0; end
            writeline(obj.tcp_, line);
            tEnd = tic;
            while toc(tEnd) < timeout
                obj.readAvailable_();
                hit = obj.takeAck_(strtrim(line));
                if ~isempty(hit)
                    isok = startsWith(hit, "ok") || startsWith(hit, "pong");
                    ack = hit;
                    return;
                end
                pause(0.01);
            end
            isok = false;
            ack = "无应答（超时）";
        end

        function [isok, ack] = drive(obj, vx, vy, wz)
            %车体速度指令 v：vx/vy m/s，wz rad/s（宿主 10Hz 重发）
            if nargin < 2, vx = 0; end
            if nargin < 3, vy = 0; end
            if nargin < 4, wz = 0; end
            [isok, ack] = obj.cmd(sprintf('v %.3f %.3f %.3f', vx, vy, wz));
        end

        function [isok, ack] = setRpm(obj, r1, r2, r3)
            %三电机输出轴目标转速指令 rpm
            if nargin < 2, r1 = 0; end
            if nargin < 3, r2 = 0; end
            if nargin < 4, r3 = 0; end
            [isok, ack] = obj.cmd(sprintf('rpm %.2f %.2f %.2f', r1, r2, r3));
        end

        function [isok, ack] = stop(obj)
            %刹车并停止速度重发（CMD_STOP）
            [isok, ack] = obj.cmd("stop");
        end

        function [isok, rttms] = ping(obj, timeout)
            %测板端 RTT；rttms 为毫秒，无应答时为 NaN
            if nargin < 2, timeout = 2.0; end
            [isok, ack] = obj.cmd("ping", timeout);
            rttms = NaN;
            if isok
                tok = regexp(ack, 'rtt=([\d.]+)ms', 'tokens', 'once');
                if ~isempty(tok), rttms = str2double(tok{1}); end
            end
        end

        function startTelemetry(obj, rateHz)
            %启动后台定时器接收遥测（默认 20Hz 轮询排空）
            if nargin < 2, rateHz = 20; end
            if obj.isTelemetryOn(), return; end
            obj.timer_ = timer("ExecutionMode", "fixedRate", "BusyMode", "drop", ...
                "Period", 1/rateHz, "TimerFcn", @(~,~) obj.pumpLines());
            start(obj.timer_);
        end

        function stopTelemetry(obj)
            if isa(obj.timer_, "timer") && isvalid(obj.timer_)
                stop(obj.timer_);
                delete(obj.timer_);
            end
            obj.timer_ = [];
        end

        function tf = isTelemetryOn(obj)
            tf = isa(obj.timer_, "timer") && isvalid(obj.timer_) ...
                 && strcmp(obj.timer_.Running, "on");
        end

        function pumpLines(obj)
            %排空当前收到的行：JSON 入各缓冲，其余行入 Messages
            %供定时器与手动轮询调用；cmd() 内部走同一读取路径
            try
                obj.readAvailable_();
            catch err
                obj.stopTelemetry();
                obj.IsConnected = false;
                warning('ChassisClient:linkLost', 'chassis_host 连接已断开: %s', err.message);
            end
        end

        function clearTelemetry(obj)
            obj.Odom = {}; obj.Enc = {}; obj.Imu = {}; obj.Telem = {};
            obj.Events = {}; obj.Messages = {}; obj.LatestStatus = [];
        end

        function tf = waitReady(obj, timeout)
            %等待链路 READY（STATUS.con=1 且 en=1），见 chassis-host/docs/protocol-serial.md
            %宿主建链后自动使能；未开遥测时自动开启
            if nargin < 2, timeout = 5; end
            if ~obj.isTelemetryOn(), obj.startTelemetry(); end
            tEnd = tic;
            while toc(tEnd) < timeout
                st = obj.LatestStatus;
                if ~isempty(st) && st.con == 1 && st.en == 1
                    tf = true; return;
                end
                pause(0.05);
            end
            tf = false;
        end

        function s = getOdom(obj)
            %Odom 缓冲的列式视图，直接可 plot：plot(o.t, o.vx)、plot(o.x, o.y)
            o = obj.Odom;
            s.t   = cellfun(@(r) r.t,   o);
            s.vx  = cellfun(@(r) r.vx,  o);
            s.vy  = cellfun(@(r) r.vy,  o);
            s.wz  = cellfun(@(r) r.wz,  o);
            s.x   = cellfun(@(r) r.x,   o);
            s.y   = cellfun(@(r) r.y,   o);
            s.yaw = cellfun(@(r) r.yaw, o);
        end

        function s = getEnc(obj)
            %编码器累计计数与转速，N x 3（三电机）
            o = obj.Enc;
            s.t     = cellfun(@(r) r.t, o);
            rows    = cellfun(@(r) r.count, o, 'UniformOutput', false);
            s.count = cat(1, rows{:});
            rows    = cellfun(@(r) r.rpm, o, 'UniformOutput', false);
            s.rpm   = cat(1, rows{:});
        end

        function s = getImu(obj)
            %IMU 加速度(g)/角速度(dps)/欧拉角(deg)，N x 3
            o = obj.Imu;
            s.t        = cellfun(@(r) r.t, o);
            rows       = cellfun(@(r) r.accel_g, o, 'UniformOutput', false);
            s.accel_g  = cat(1, rows{:});
            rows       = cellfun(@(r) r.gyro_dps, o, 'UniformOutput', false);
            s.gyro_dps = cat(1, rows{:});
            rows       = cellfun(@(r) r.rpy_deg, o, 'UniformOutput', false);
            s.rpy_deg  = cat(1, rows{:});
        end

        function s = getTelem(obj)
            %三电机目标转速/实际转速/输出占空比，N x 3
            o = obj.Telem;
            s.t       = cellfun(@(r) r.t, o);
            rows      = cellfun(@(r) r.tgt_rpm, o, 'UniformOutput', false);
            s.tgt_rpm = cat(1, rows{:});
            rows      = cellfun(@(r) r.rpm, o, 'UniformOutput', false);
            s.rpm     = cat(1, rows{:});
            rows      = cellfun(@(r) r.out_pct, o, 'UniformOutput', false);
            s.out_pct = cat(1, rows{:});
        end

        function delete(obj)
            if isa(obj.timer_, "timer") && isvalid(obj.timer_)
                delete(obj.timer_);
            end
            if ~isempty(obj.tcp_) && isvalid(obj.tcp_)
                delete(obj.tcp_);
            end
        end
    end

    methods (Access = private)
        function t = now_(obj)
            t = toc(obj.t0_);
        end

        function readAvailable_(obj)
            %排空 socket 并按行分流（JSON/应答/杂项）
            %逐字节自缓冲而非 readline：readline 遇半行残包会阻塞到超时
            if isempty(obj.tcp_) || ~isvalid(obj.tcp_), return; end
            n = obj.tcp_.NumBytesAvailable;
            if n > 0
                d = read(obj.tcp_, n, "uint8");
                obj.rxbuf_ = [obj.rxbuf_, d(:).'];
            end
            cut = find(obj.rxbuf_ == uint8(10), 1, "last");
            if isempty(cut), return; end
            chunk = char(obj.rxbuf_(1:cut));
            obj.rxbuf_ = obj.rxbuf_(cut+1:end);
            parts = regexp(chunk, '\r?\n', 'split');
            % 索引遍历并解包：for p = parts 迭代出的 p 是 1x1 cell 而非行内容
            for i = 1:numel(parts)
                p = parts{i};
                if isempty(p), continue; end
                obj.route_(p);
            end
        end

        function route_(obj, line)
            %按行类型分流：ok/err/pong 入应答队列，JSON 入遥测缓冲，其余入 Messages
            if startsWith(line, "ok") || startsWith(line, "err") || startsWith(line, "pong")
                obj.ackQueue_{end+1} = line;
                return;
            end
            if line(1) == '{'
                try
                    js = jsondecode(line);
                catch
                    js = [];  % 坏行落入 Messages 便于排查
                end
                if isstruct(js) && isfield(js, 't')
                    obj.store_(js);
                    return;
                end
            end
            obj.Messages{end+1} = struct('t', obj.now_(), 'text', line);
        end

        function line = takeAck_(obj, cmdline)
            %从应答队列取走首个匹配行：ok/err 直接命中，pong 只应答 ping
            line = '';
            for i = 1:numel(obj.ackQueue_)
                l = obj.ackQueue_{i};
                if startsWith(l, "ok") || startsWith(l, "err") || ...
                        (startsWith(l, "pong") && startsWith(cmdline, "ping"))
                    line = l;
                    obj.ackQueue_(i) = [];
                    return;
                end
            end
        end

        function store_(obj, js)
            t = obj.now_();
            % jsondecode 把一维 JSON 数组解成列向量，入库统一转 1x3 行向量
            switch js.t
                case 'odom'
                    obj.Odom{end+1} = struct('t',t, 'vx',js.vx, 'vy',js.vy, ...
                        'wz',js.wz, 'x',js.x, 'y',js.y, 'yaw',js.yaw);
                case 'enc'
                    obj.Enc{end+1} = struct('t',t, 'count',js.count(:).', ...
                        'rpm',js.rpm(:).');
                case 'imu'
                    obj.Imu{end+1} = struct('t',t, 'accel_g',js.accel_g(:).', ...
                        'gyro_dps',js.gyro_dps(:).', 'rpy_deg',js.rpy_deg(:).');
                case 'telem'
                    obj.Telem{end+1} = struct('t',t, 'tgt_rpm',js.tgt_rpm(:).', ...
                        'rpm',js.rpm(:).', 'out_pct',js.out_pct(:).');
                case 'status'
                    obj.LatestStatus = struct('t',t, 'uptime',js.uptime, ...
                        'imu_ok',js.imu_ok, 'en',js.en, 'mode',js.mode, 'con',js.con);
                otherwise
                    obj.Events{end+1} = struct('t',t, 'type',string(js.t), 'data',js);
            end
        end
    end
end
