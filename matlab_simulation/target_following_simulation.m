function target_following_simulation()
% TARGET_FOLLOWING_SIMULATION Simulates Freenove 4WD following a moving target
%
% This script simulates a differential drive robot following a target
% using PID controllers for distance and angle control.

    close all;
    clear;
    clc;
    
    %% Simulation Parameters
    dt = 0.05;  % Time step (s)
    t_end = 30;  % Simulation duration (s)
    time = 0:dt:t_end;
    
    %% Vehicle Parameters
    wheel_base = 0.15;  % Distance between wheels (m) - typical for Freenove 4WD
    max_velocity = 0.3;  % Maximum wheel velocity (m/s)
    
    %% Initial Conditions
    % Robot initial position and orientation
    robot_x = 0;
    robot_y = 0;
    robot_theta = 0;
    
    % Target initial position
    target_x = 2;
    target_y = 2;
    
    %% Controller Parameters
    % Distance controller (controls forward speed)
    Kp_dist = 0.5;
    Ki_dist = 0.01;
    Kd_dist = 0.1;
    
    % Angle controller (controls turning)
    Kp_angle = 1.5;
    Ki_angle = 0.01;
    Kd_angle = 0.2;
    
    % Create PID controllers
    distance_controller = pid_controller(Kp_dist, Ki_dist, Kd_dist, max_velocity);
    angle_controller = pid_controller(Kp_angle, Ki_angle, Kd_angle, max_velocity);
    
    % Desired following distance
    desired_distance = 0.5;  % meters
    
    %% Storage for plotting
    robot_trajectory_x = zeros(length(time), 1);
    robot_trajectory_y = zeros(length(time), 1);
    target_trajectory_x = zeros(length(time), 1);
    target_trajectory_y = zeros(length(time), 1);
    distance_errors = zeros(length(time), 1);
    angle_errors = zeros(length(time), 1);
    
    %% Simulation Loop
    figure('Name', 'Target Following Simulation', 'Position', [100, 100, 1200, 500]);
    
    for i = 1:length(time)
        t = time(i);
        
        % Update target position (circular motion for demonstration)
        target_x = 2 + 1.5 * cos(0.3 * t);
        target_y = 2 + 1.5 * sin(0.3 * t);
        
        % Calculate distance and angle to target
        dx = target_x - robot_x;
        dy = target_y - robot_y;
        distance = sqrt(dx^2 + dy^2);
        angle_to_target = atan2(dy, dx);
        
        % Calculate angle error (normalized to [-pi, pi])
        angle_error = angle_to_target - robot_theta;
        angle_error = atan2(sin(angle_error), cos(angle_error));
        
        % Calculate distance error
        distance_error = distance - desired_distance;
        
        % Compute control outputs
        v_forward = distance_controller.compute(distance_error, dt);
        v_turn = angle_controller.compute(angle_error, dt);
        
        % Convert to wheel velocities
        v_left = v_forward - v_turn;
        v_right = v_forward + v_turn;
        
        % Apply velocity limits
        v_left = max(min(v_left, max_velocity), -max_velocity);
        v_right = max(min(v_right, max_velocity), -max_velocity);
        
        % Update robot position using vehicle model
        [robot_x, robot_y, robot_theta] = vehicle_model(robot_x, robot_y, robot_theta, ...
                                                         v_left, v_right, dt, wheel_base);
        
        % Store data for plotting
        robot_trajectory_x(i) = robot_x;
        robot_trajectory_y(i) = robot_y;
        target_trajectory_x(i) = target_x;
        target_trajectory_y(i) = target_y;
        distance_errors(i) = distance_error;
        angle_errors(i) = angle_error;
        
        % Real-time plotting (every 10 steps to reduce overhead)
        if mod(i, 10) == 0 || i == 1
            % Trajectory plot
            subplot(1, 3, 1);
            plot(target_trajectory_x(1:i), target_trajectory_y(1:i), 'r--', 'LineWidth', 2);
            hold on;
            plot(robot_trajectory_x(1:i), robot_trajectory_y(1:i), 'b-', 'LineWidth', 2);
            plot(target_x, target_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            plot(robot_x, robot_y, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
            
            % Draw robot orientation
            arrow_length = 0.3;
            quiver(robot_x, robot_y, arrow_length*cos(robot_theta), ...
                   arrow_length*sin(robot_theta), 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
            
            hold off;
            xlabel('X (m)');
            ylabel('Y (m)');
            title('Robot and Target Trajectories');
            legend('Target', 'Robot', 'Location', 'best');
            grid on;
            axis equal;
            
            % Distance error plot
            subplot(1, 3, 2);
            plot(time(1:i), distance_errors(1:i), 'b-', 'LineWidth', 1.5);
            hold on;
            plot([0, t_end], [0, 0], 'k--');
            hold off;
            xlabel('Time (s)');
            ylabel('Distance Error (m)');
            title('Distance Error vs Time');
            grid on;
            xlim([0, t_end]);
            
            % Angle error plot
            subplot(1, 3, 3);
            plot(time(1:i), rad2deg(angle_errors(1:i)), 'r-', 'LineWidth', 1.5);
            hold on;
            plot([0, t_end], [0, 0], 'k--');
            hold off;
            xlabel('Time (s)');
            ylabel('Angle Error (deg)');
            title('Angle Error vs Time');
            grid on;
            xlim([0, t_end]);
            
            drawnow;
        end
    end
    
    %% Performance Metrics
    fprintf('\n=== Simulation Results ===\n');
    fprintf('Average distance error: %.3f m\n', mean(abs(distance_errors)));
    fprintf('Max distance error: %.3f m\n', max(abs(distance_errors)));
    fprintf('Final distance error: %.3f m\n', abs(distance_errors(end)));
    fprintf('Average angle error: %.2f deg\n', mean(abs(rad2deg(angle_errors))));
    fprintf('Max angle error: %.2f deg\n', max(abs(rad2deg(angle_errors))));
    fprintf('Final angle error: %.2f deg\n', abs(rad2deg(angle_errors(end))));
    
    % Save figure
    saveas(gcf, 'target_following_results.png');
    fprintf('\nFigure saved as target_following_results.png\n');
end
