classdef Vehicle2DConfig
    properties
        wheelbase
        track_width
        steer_max
        mass
        Izz
        z_cg
        a
        r_wheel
        I_wheel
        Tmax
        Tzero
        Tnegmax
        Pmax
        Pnegmax
        gear_ratio
        brake_Tmax
        brake_bias
        drivetrain_type
        differential_type
        diff_damping
        tire_config
        CDx
        CMz
        Af
        rho
    end

    methods
        function obj = Vehicle2DConfig(filename_or_wheelbase_, track_width_, steer_max_, ...
                mass_, Izz_, ...
                z_cg_, a_, r_wheel_, I_wheel_, ...
                Tmax_, Tzero_, Tnegmax_, Pmax_, Pnegmax_, gear_ratio_, ...
                brake_Tmax_, brake_bias_, ...
                drivetrain_type_, ...
                differential_type_, diff_damping_, ...
                tire_config_, ...
                CDx_, CMz_, Af_, rho_)

            if nargin == 1
                if strcmp(filename_or_wheelbase_, 'sym')
                    obj.tire_config = TireConfig('sym');
                    obj.drivetrain_type = sym(0);
                    obj.differential_type = sym(1);
                    fns = string(fieldnames(obj))';
                    fns = fns(~(fns == "tire_config" | fns == "drivetrain_type" | fns == "differential_type"));
                    for fn = fns
                        obj.(fn) = sym(fn);
                    end
                else
                    obj = Vehicle2DConfig.loadFromFile(filename_or_wheelbase_);
                end
            else
                obj.wheelbase = filename_or_wheelbase_;
                obj.track_width = track_width_;
                obj.steer_max = steer_max_;
                obj.mass = mass_;
                obj.Izz = Izz_;
                obj.z_cg = z_cg_;
                obj.a = a_;
                obj.r_wheel = r_wheel_;
                obj.I_wheel = I_wheel_;
                obj.Tmax = Tmax_;
                obj.Tzero = Tzero_;
                obj.Tnegmax = Tnegmax_;
                obj.Pmax = Pmax_;
                obj.Pnegmax = Pnegmax_;
                obj.gear_ratio = gear_ratio_;
                obj.brake_Tmax = brake_Tmax_;
                obj.brake_bias = brake_bias_;
                obj.drivetrain_type = drivetrain_type_;
                % 0 = obj.drivetrain_type = "RWD";
                % 1 = obj.drivetrain_type = "AWD";
                % 2 = obj.drivetrain_type = "FWD";
                
                obj.differential_type = differential_type_;
                % 0 = obj.differential_type = "DIFF_OPEN";
                % 1 = obj.differential_type = "DIFF_LOCKED";

                obj.diff_damping = diff_damping_;
                obj.tire_config = TireConfig(tire_config_);
                obj.CDx = CDx_;
                obj.CMz = CMz_;
                obj.Af = Af_;
                obj.rho = rho_;
            end
        end
        
    end

    methods (Static)
        function obj = loadFromFile(json_path)
            fid = fopen(json_path); 
            raw = fread(fid,inf); 
            str = char(raw'); 
            fclose(fid); 
            config = jsondecode(str);

            obj = Vehicle2DConfig( ...
                config.wheelbase, config.track_width, config.steer_max, ...
                config.mass, config.Izz, ...
                config.z_cg, config.a, config.r_wheel, config.I_wheel, ...
                config.Tmax, config.Tzero, config.Tnegmax, ...
                config.Pmax, config.Pnegmax, config.gear_ratio, ...
                config.brake_Tmax, config.brake_bias, ...
                config.drivetrain_type, ...
                config.differential_type, config.diff_damping, ...
                config.tire_config, ...
                config.CDx, config.CMz, config.Af, config.rho);
        end
    end
end