classdef SbrSerialIO < matlab.System
%SBRSERIALIO  Simulink block: bidirectional exchange with the motor node.
%
%   Use it through the "MATLAB System" block (simulink/User-Defined
%   Functions/MATLAB System) with "Simulate using" = INTERPRETED EXECUTION.
%   Code generation is NOT supported (a serialport object is simulation-only)
%   and is not needed: when you deploy, the controller is generated for the
%   ESP32 and this block disappears from the model.
%
%   Input  : u        — torque command (scalar)
%   Output : y (4x1)  — [angle_rad; rate_rads; mean_wheel_position_rad; ok]
%                       ok = 1 when the last frame is valid and fault-free
%
%   The block arms the motors when the simulation starts and stops them when
%   it ends.

    properties (Nontunable)
        Port     = "COM7"
        Baud     = 115200
        UMax     = 2.0        % command limit pushed to the firmware
        TiltMax  = 0.35       % [rad] safety cut-off
        BootWait = 6.0        % [s] wait after the ESP32 resets on port open
    end

    properties
        Trim = 0.0            % [rad] equilibrium point (tunable)
    end

    properties (Access = private)
        lnk
        lastY = [0; 0; 0; 0]
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.lnk = SbrLink(obj.Port, obj.Baud, obj.BootWait);
            obj.lnk.trim(obj.Trim);
            obj.lnk.limits(obj.UMax, obj.TiltMax);
            obj.lnk.setpoint(0);
            obj.lnk.mode(1);          % MODE_DIRECT: the controller is in Simulink
            obj.lnk.flushInput();
            obj.lnk.arm(true);
            obj.lastY = [0; 0; 0; 0];
        end

        function y = stepImpl(obj, u)
            obj.lnk.torque(u, u);

            d = obj.lnk.readLatest();
            if isempty(d)
                y = obj.lastY;                 % hold the last measurement
                y(4) = 0;                      % but flag "no fresh data"
                return
            end

            ok = double(d.imu_ok && ~d.tilt_fault && ~d.wdt_fault);
            y  = [d.angle; d.rate; (d.pos0 + d.pos1)/2; ok];
            obj.lastY = y;
        end

        function releaseImpl(obj)
            try
                obj.lnk.stop();
            catch
            end
            delete(obj.lnk);
            obj.lnk = [];
        end

        % ---- Simulink interface ---------------------------------------------
        function n = getNumInputsImpl(~),  n = 1; end
        function n = getNumOutputsImpl(~), n = 1; end
        function sz = getOutputSizeImpl(~),        sz = [4 1];   end
        function dt = getOutputDataTypeImpl(~),    dt = "double"; end
        function c  = isOutputComplexImpl(~),      c  = false;   end
        function f  = isOutputFixedSizeImpl(~),    f  = true;    end
        function flag = isInactivePropertyImpl(~, ~), flag = false; end
    end
end
