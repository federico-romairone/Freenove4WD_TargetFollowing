function [x_new, y_new, theta_new] = vehicle_model(x, y, theta, v_left, v_right, dt, wheel_base)
% VEHICLE_MODEL Differential drive kinematic model for Freenove 4WD
%
% Inputs:
%   x, y, theta - Current position and orientation
%   v_left, v_right - Left and right wheel velocities (m/s)
%   dt - Time step (s)
%   wheel_base - Distance between left and right wheels (m)
%
% Outputs:
%   x_new, y_new, theta_new - Updated position and orientation

    % Calculate linear and angular velocities
    v = (v_left + v_right) / 2;  % Forward velocity
    omega = (v_right - v_left) / wheel_base;  % Angular velocity
    
    % Update position using Euler integration
    x_new = x + v * cos(theta) * dt;
    y_new = y + v * sin(theta) * dt;
    theta_new = theta + omega * dt;
    
    % Normalize angle to [-pi, pi]
    theta_new = atan2(sin(theta_new), cos(theta_new));
end
