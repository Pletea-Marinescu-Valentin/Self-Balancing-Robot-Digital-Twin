classdef SbrLink < handle
%SBRLINK  Link to the motor node, over USB serial or WiFi TCP.
%   Protocol: docs/protocol.md
%
%   lnk = SbrLink("COM4");            % USB
%   lnk = SbrLink("192.168.4.1");     % the robot's own access point
%   lnk.arm(true);  lnk.mode(1);
%   d = lnk.readLatest();
%   lnk.torque(0.5, 0.5);
%   lnk.stop();  clear lnk
%
%   readLatest() returns the NEWEST valid frame and discards the backlog.

    properties (Constant, Access = private)
        PKT_LEN = 88
        CMD_LEN = 16
        SYNC0   = uint8(165)
        SYNC1   = uint8(90)
        MAXBUF  = 88 * 12
    end

    properties (SetAccess = private)
        port
        isTcp     (1,1) logical = false
        nDropped  (1,1) double = 0
        nBadCrc   (1,1) double = 0
    end

    properties (Access = private)
        buf uint8 = uint8([])
    end

    methods
        function obj = SbrLink(target, arg2, bootWait)
        %SBRLINK  Open the link. Serial or WiFi, chosen by what you pass.
        %   SbrLink("COM4")            serial, 115200
        %   SbrLink("192.168.4.1")     TCP to the robot's own access point
        %   arg2 is the baud rate for a COM port, the TCP port for an address.
            arguments
                target   (1,1) string
                arg2     (1,1) double = 0
                bootWait (1,1) double = -1
            end

            obj.isTcp = contains(target, ".");
            if obj.isTcp
                if arg2 == 0,     arg2 = 3333;     end
                if bootWait < 0,  bootWait = 0.5;  end
                obj.port = tcpclient(target, arg2, "Timeout", 0.05);
            else
                if arg2 == 0,     arg2 = 115200;   end
                if bootWait < 0,  bootWait = 6.0;  end
                obj.port = serialport(target, arg2, "Timeout", 0.05);
            end

            pause(bootWait);
            obj.flushInput();
        end

        function delete(obj)
            try
                obj.stop();
            catch
            end
            obj.port = [];
        end

        function d = readLatest(obj)
        %READLATEST  Newest valid frame, or [] if nothing new arrived.
        %   A dropped TCP connection must read as "no frame", not as an
        %   exception: over WiFi that would abort a tuning session on one
        %   bad moment, while the robot is still balancing perfectly well.
            d = [];
            try
                n = obj.port.NumBytesAvailable;
                if n > 0
                    obj.buf = [obj.buf; uint8(read(obj.port, n, "uint8")')];
                end
            catch
                return
            end
            if numel(obj.buf) < obj.PKT_LEN
                return
            end

            L = obj.PKT_LEN;
            found = 0;
            for i = (numel(obj.buf) - L + 1) : -1 : 1
                if obj.buf(i) == obj.SYNC0 && obj.buf(i+1) == obj.SYNC1
                    p = obj.buf(i : i+L-1);
                    if SbrLink.xorSum(p(3:end-1)) == p(end)
                        d = SbrLink.decode(p);
                        found = i;
                        break
                    end
                end
            end

            if found > 0
                obj.nDropped = obj.nDropped + floor((found - 1) / L);
                obj.buf = obj.buf(found + L : end);
            elseif numel(obj.buf) > obj.MAXBUF
                obj.nBadCrc = obj.nBadCrc + 1;
                obj.buf = obj.buf(end - obj.PKT_LEN + 1 : end);
            end
        end

        function flushInput(obj)
            try
                flush(obj.port);
            catch
                n = obj.port.NumBytesAvailable;
                if n > 0, read(obj.port, n, "uint8"); end
            end
            obj.buf = uint8([]);
        end

        function info = diagnose(obj, secs)
        %DIAGNOSE  Blocks until a valid frame or timeout. Returns .frame,
        %   .nBytes, .text and .elapsed, which separate "board silent" from
        %   "board printing an error" from "wrong baud".
            arguments
                obj
                secs (1,1) double = 12
            end
            info = struct('frame', [], 'nBytes', 0, 'text', "", 'elapsed', 0);
            L    = obj.PKT_LEN;
            HEAD = 4096;
            WIN  = 4 * L;
            head = uint8([]);
            win  = uint8([]);
            t = tic;

            while toc(t) < secs
                n = obj.port.NumBytesAvailable;
                if n == 0
                    pause(0.01);
                    continue
                end
                chunk = uint8(read(obj.port, n, "uint8")');
                info.nBytes = info.nBytes + numel(chunk);

                if numel(head) < HEAD
                    head = [head; chunk(1 : min(end, HEAD - numel(head)))]; %#ok<AGROW>
                end
                win = [win; chunk]; %#ok<AGROW>
                if numel(win) > WIN, win = win(end - WIN + 1 : end); end

                for i = 1 : max(0, numel(win) - L + 1)
                    if win(i) == obj.SYNC0 && win(i+1) == obj.SYNC1
                        p = win(i : i+L-1);
                        if SbrLink.xorSum(p(3:end-1)) == p(end)
                            info.frame = SbrLink.decode(p);
                            break
                        end
                    end
                end
                if ~isempty(info.frame), break; end
            end

            info.elapsed = toc(t);
            str = char(head(:)');
            str((head < 32 | head > 126)') = ' ';
            hits = regexp(string(str), '[\x20-\x7E]{8,}', 'match');
            if ~isempty(hits)
                info.text = strtrim(strjoin(unique(hits, 'stable'), ' | '));
            end
        end

        function torque(obj, u0, u1),      obj.send(1, u0, u1, 0);        end
        function gains(obj, kp, ki, kd),   obj.send(2, kp, ki, kd);       end
        function mode(obj, m),             obj.send(3, m, 0, 0);          end
        function setpoint(obj, a),         obj.send(4, a, 0, 0);          end
        function limits(obj, umax, tmax),  obj.send(5, umax, tmax, 0);    end
        function arm(obj, on),             obj.send(6, double(on), 0, 0); end
        function trim(obj, t),             obj.send(7, t, 0, 0);          end

        function imuAxis(obj, angIdx, gyrIdx, angSign, gyrSign)
        %IMUAXIS  Select the balance channel at runtime.
        %   angIdx / gyrIdx : 0..2  (Euler: 0 heading, 1 roll, 2 pitch;
        %                            gyro:  0 x, 1 y, 2 z)
        %   The two signs are independent: the Euler and gyro conventions do
        %   not always agree on the same axis.
            arguments
                obj
                angIdx  (1,1) double
                gyrIdx  (1,1) double
                angSign (1,1) double = 1
                gyrSign (1,1) double = 1
            end
            obj.send(8, sign(angSign) * (angIdx + 1), ...
                        sign(gyrSign) * (gyrIdx + 1), 0);
        end

        function heartbeat(obj)
            obj.send(1, 0, 0, 0);
        end

        function stop(obj)
            obj.send(1, 0, 0, 0);
            obj.send(3, 0, 0, 0);
            obj.send(6, 0, 0, 0);
        end
    end

    methods (Access = private)
        function send(obj, type, p1, p2, p3)
            pkt = zeros(1, obj.CMD_LEN, "uint8");
            pkt(1) = obj.SYNC0;
            pkt(2) = obj.SYNC1;
            pkt(3) = uint8(type);
            pkt(4:15) = typecast(single([p1 p2 p3]), "uint8");
            pkt(16) = SbrLink.xorSum(pkt(3:15));
            try
                write(obj.port, pkt, "uint8");
            catch
            end
        end
    end

    methods (Static, Hidden)
        function c = xorSum(b)
            c = uint8(0);
            for k = 1:numel(b)
                c = bitxor(c, b(k));
            end
        end

        function d = decode(p)
            p  = p(:);
            f  = double(typecast(p(7:86), "single"));
            st = p(87);
            d = struct( ...
                't_us',       double(typecast(p(3:6), "uint32")), ...
                'eul',        f(1:3)',   ...
                'gyr',        f(4:6)',   ...
                'acc',        f(7:9)',   ...
                'angle',      f(10), ...
                'rate',       f(11), ...
                'pos0',       f(12), 'pos1', f(13), ...
                'vel0',       f(14), 'vel1', f(15), ...
                'u0',         f(16), 'u1',   f(17), ...
                'foc_hz',     f(18), ...
                'imu_hz',     f(19), ...
                'imu_age_ms', f(20), ...
                'armed',      bitand(st, 1)  > 0, ...
                'imu_ok',     bitand(st, 2)  > 0, ...
                'tilt_fault', bitand(st, 4)  > 0, ...
                'wdt_fault',  bitand(st, 8)  > 0, ...
                'imu_fault',  bitand(st, 16) > 0, ...
                'imu_link',   bitand(st, 32) > 0, ...
                'mode',       double(bitshift(st, -6)));
        end
    end
end
