classdef trajectory_generator < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    % Untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function yaw_angle = stepImpl(obj,current_point,target_point,v_x)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            if target_point(1,1)<=current_point(1,1)
                if target_point(1,2)<= current_point(1,2)
                    yaw_angle=0;
                    return;
                else
                    yaw_angle = 0.02;
                    return;
                end
                
            end
            
            %% Waypoint generation
            dt = 0.1;
            T = abs(target_point(1,1)-current_point(1,1))/v_x;
            
            x_f = target_point(1,1); % Final x coordinate
            y_f = target_point(1,2); % Final y coordinate
            
            %% Vehicle next state in x direction (Longitudinal)
            
            x=current_point(1,1) + dt*v_x;
            x_dot=v_x;
            x_ddot=0;
            
            nextState_x=[x;x_dot;x_ddot];
            
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            
            a0=current_point(1,2);
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            
            %% Vehicle states in y direction
            y=[];
            y_dot=[];
            y_ddot=[];
            
            t = dt;
            y       = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
            y_dot   = a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4;
            y_ddot  = 2*a2+6*a3*t+12*a4*t^2+20*a5*t^3;
            
            nextState_y=[y;y_dot;y_ddot];
            
            %% Yaw angle reference
            yaw_angle=atan(y_dot/ x_dot);
            
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", 0.1);

            % Example: specify discrete sample time
            % sts = obj.createSampleTime("Type", "Discrete", ...
            %     "SampleTime", 1);
        end
    end
end
