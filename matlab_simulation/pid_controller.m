classdef pid_controller < handle
% PID_CONTROLLER A simple PID controller implementation
%
% Properties:
%   Kp, Ki, Kd - PID gains
%   integral - Integral term accumulator
%   prev_error - Previous error for derivative calculation
%   output_limit - Maximum absolute output value
    
    properties
        Kp
        Ki
        Kd
        integral
        prev_error
        output_limit
    end
    
    methods
        function obj = pid_controller(Kp, Ki, Kd, output_limit)
            % Constructor
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.integral = 0;
            obj.prev_error = 0;
            obj.output_limit = output_limit;
        end
        
        function output = compute(obj, error, dt)
            % Compute PID control output
            
            % Proportional term
            P = obj.Kp * error;
            
            % Integral term with anti-windup
            obj.integral = obj.integral + error * dt;
            % Anti-windup: limit integral term
            max_integral = obj.output_limit / obj.Ki;
            obj.integral = max(min(obj.integral, max_integral), -max_integral);
            I = obj.Ki * obj.integral;
            
            % Derivative term
            if dt > 0
                derivative = (error - obj.prev_error) / dt;
            else
                derivative = 0;
            end
            D = obj.Kd * derivative;
            
            % Calculate total output
            output = P + I + D;
            
            % Apply output limits
            output = max(min(output, obj.output_limit), -obj.output_limit);
            
            % Store error for next iteration
            obj.prev_error = error;
        end
        
        function reset(obj)
            % Reset controller state
            obj.integral = 0;
            obj.prev_error = 0;
        end
    end
end
