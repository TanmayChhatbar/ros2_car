classdef TireConfig
    properties
        B
        C
        D
        E
        f
        coeff_rr
        coeff_stiction
    end

    methods
        function obj = TireConfig(s)
            if strcmp(s, 'sym')
                for fn = string(fieldnames(obj))'
                    obj.(fn) = sym(fn);
                end
            else
                obj.B = s.B;
                obj.C = s.C;
                obj.D = s.D;
                obj.E = s.E;
                obj.f = s.f;
                obj.coeff_rr = s.coeff_rr;
                obj.coeff_stiction = s.coeff_stiction;
            end
        end

        function [Fx_wheel, Fy_wheel] = calcTireForces(obj, slip_angle, slip_ratio, w_wheel, Fz_wheel)
            if isa(slip_angle, 'sym')
                slip_angle_corr = slip_angle;
            else
                slip_angle_corr = slip_angle * min(1, abs(w_wheel)*0.01);
            end
            slipNet = sqrt(slip_angle_corr^2 + slip_ratio^2 * obj.f^2);
            if ~isa(slipNet, 'sym') && slipNet < 1e-9
                Fx_wheel = 0;
                Fy_wheel = 0;
            else
                Fnet = obj.D * sin(obj.C * atan(obj.B*slipNet - obj.E * (obj.B * slipNet - atan(obj.B * slipNet))));
                F_rolling_resistance = obj.coeff_rr * Fz_wheel;
                if isa(slip_angle, 'sym')
                    F_rolling_stiction = obj.coeff_stiction * Fz_wheel * sign(w_wheel);
                else
                    F_rolling_stiction = obj.coeff_stiction * Fz_wheel * sign(w_wheel) * min(abs(w_wheel)*100, 1);
                end
                F_partial = Fz_wheel * Fnet / slipNet;
                Fx_wheel = F_partial * (slip_ratio * obj.f) - F_rolling_resistance - F_rolling_stiction;
                Fy_wheel = - F_partial * slip_angle_corr;
            end
        end
    end
end