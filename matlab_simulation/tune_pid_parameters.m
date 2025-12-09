function tune_pid_parameters()
% TUNE_PID_PARAMETERS Interactive tool for tuning PID controller parameters
%
% This script allows you to test different PID parameters and visualize
% the system response to help optimize controller performance.

    close all;
    clear;
    clc;
    
    fprintf('=== PID Parameter Tuning Tool ===\n\n');
    fprintf('This tool helps you find optimal PID parameters for the target following system.\n');
    fprintf('Edit the parameter sets below and run simulations to compare performance.\n\n');
    
    %% Test Parameter Sets
    % Define multiple parameter sets to test
    param_sets = {
        % [Kp_dist, Ki_dist, Kd_dist, Kp_angle, Ki_angle, Kd_angle, Name]
        struct('Kp_dist', 0.3, 'Ki_dist', 0.005, 'Kd_dist', 0.05, ...
               'Kp_angle', 1.0, 'Ki_angle', 0.005, 'Kd_angle', 0.1, 'name', 'Conservative'),
        struct('Kp_dist', 0.5, 'Ki_dist', 0.01, 'Kd_dist', 0.1, ...
               'Kp_angle', 1.5, 'Ki_angle', 0.01, 'Kd_angle', 0.2, 'name', 'Balanced'),
        struct('Kp_dist', 0.7, 'Ki_dist', 0.02, 'Kd_dist', 0.15, ...
               'Kp_angle', 2.0, 'Ki_angle', 0.02, 'Kd_angle', 0.3, 'name', 'Aggressive')
    };
    
    %% Run simulations for each parameter set
    results = cell(length(param_sets), 1);
    
    for i = 1:length(param_sets)
        fprintf('Running simulation %d: %s\n', i, param_sets{i}.name);
        results{i} = run_simulation_with_params(param_sets{i});
    end
    
    %% Plot comparison
    figure('Name', 'PID Parameter Comparison', 'Position', [100, 100, 1400, 800]);
    
    % Distance error comparison
    subplot(2, 2, 1);
    hold on;
    for i = 1:length(results)
        plot(results{i}.time, results{i}.distance_errors, 'LineWidth', 1.5, ...
             'DisplayName', param_sets{i}.name);
    end
    hold off;
    xlabel('Time (s)');
    ylabel('Distance Error (m)');
    title('Distance Error Comparison');
    legend('Location', 'best');
    grid on;
    
    % Angle error comparison
    subplot(2, 2, 2);
    hold on;
    for i = 1:length(results)
        plot(results{i}.time, rad2deg(results{i}.angle_errors), 'LineWidth', 1.5, ...
             'DisplayName', param_sets{i}.name);
    end
    hold off;
    xlabel('Time (s)');
    ylabel('Angle Error (deg)');
    title('Angle Error Comparison');
    legend('Location', 'best');
    grid on;
    
    % Trajectory comparison
    subplot(2, 2, 3);
    hold on;
    plot(results{1}.target_x, results{1}.target_y, 'k--', 'LineWidth', 2, ...
         'DisplayName', 'Target');
    for i = 1:length(results)
        plot(results{i}.robot_x, results{i}.robot_y, 'LineWidth', 1.5, ...
             'DisplayName', param_sets{i}.name);
    end
    hold off;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Trajectory Comparison');
    legend('Location', 'best');
    grid on;
    axis equal;
    
    % Performance metrics
    subplot(2, 2, 4);
    axis off;
    
    y_pos = 0.9;
    text(0.1, y_pos, 'Performance Metrics:', 'FontSize', 12, 'FontWeight', 'bold');
    y_pos = y_pos - 0.1;
    
    for i = 1:length(results)
        y_pos = y_pos - 0.05;
        text(0.1, y_pos, sprintf('%s:', param_sets{i}.name), 'FontSize', 10, 'FontWeight', 'bold');
        y_pos = y_pos - 0.04;
        text(0.15, y_pos, sprintf('Avg Dist Error: %.3f m', results{i}.avg_dist_error), 'FontSize', 9);
        y_pos = y_pos - 0.04;
        text(0.15, y_pos, sprintf('Avg Angle Error: %.2f deg', results{i}.avg_angle_error), 'FontSize', 9);
        y_pos = y_pos - 0.04;
        text(0.15, y_pos, sprintf('Settling Time: %.2f s', results{i}.settling_time), 'FontSize', 9);
        y_pos = y_pos - 0.05;
    end
    
    saveas(gcf, 'pid_tuning_comparison.png');
    fprintf('\nComparison plot saved as pid_tuning_comparison.png\n');
    
    %% Display recommendations
    fprintf('\n=== Tuning Recommendations ===\n');
    fprintf('1. If the system oscillates: Reduce Kp, increase Kd\n');
    fprintf('2. If the system is too slow: Increase Kp\n');
    fprintf('3. If there is steady-state error: Increase Ki (carefully)\n');
    fprintf('4. For better angle tracking: Adjust Kp_angle and Kd_angle\n');
    fprintf('5. For better distance tracking: Adjust Kp_dist and Kd_dist\n');
end

function result = run_simulation_with_params(params)
    % Run a single simulation with given parameters
    
    dt = 0.05;
    t_end = 20;
    time = 0:dt:t_end;
    
    wheel_base = 0.15;
    max_velocity = 0.3;
    desired_distance = 0.5;
    
    % Initial conditions
    robot_x = 0;
    robot_y = 0;
    robot_theta = 0;
    
    % Create controllers
    distance_controller = pid_controller(params.Kp_dist, params.Ki_dist, ...
                                         params.Kd_dist, max_velocity);
    angle_controller = pid_controller(params.Kp_angle, params.Ki_angle, ...
                                      params.Kd_angle, max_velocity);
    
    % Storage
    robot_x_hist = zeros(length(time), 1);
    robot_y_hist = zeros(length(time), 1);
    target_x_hist = zeros(length(time), 1);
    target_y_hist = zeros(length(time), 1);
    distance_errors = zeros(length(time), 1);
    angle_errors = zeros(length(time), 1);
    
    % Simulation loop
    for i = 1:length(time)
        t = time(i);
        
        % Moving target
        target_x = 2 + 1.5 * cos(0.3 * t);
        target_y = 2 + 1.5 * sin(0.3 * t);
        
        % Calculate errors
        dx = target_x - robot_x;
        dy = target_y - robot_y;
        distance = sqrt(dx^2 + dy^2);
        angle_to_target = atan2(dy, dx);
        angle_error = angle_to_target - robot_theta;
        angle_error = atan2(sin(angle_error), cos(angle_error));
        distance_error = distance - desired_distance;
        
        % Control
        v_forward = distance_controller.compute(distance_error, dt);
        v_turn = angle_controller.compute(angle_error, dt);
        
        v_left = max(min(v_forward - v_turn, max_velocity), -max_velocity);
        v_right = max(min(v_forward + v_turn, max_velocity), -max_velocity);
        
        % Update
        [robot_x, robot_y, robot_theta] = vehicle_model(robot_x, robot_y, robot_theta, ...
                                                         v_left, v_right, dt, wheel_base);
        
        % Store
        robot_x_hist(i) = robot_x;
        robot_y_hist(i) = robot_y;
        target_x_hist(i) = target_x;
        target_y_hist(i) = target_y;
        distance_errors(i) = distance_error;
        angle_errors(i) = angle_error;
    end
    
    % Calculate metrics
    avg_dist_error = mean(abs(distance_errors(end-100:end)));
    avg_angle_error = mean(abs(rad2deg(angle_errors(end-100:end))));
    
    % Settling time (when error stays within 10% of desired for 2 seconds)
    settling_threshold = 0.05;  % 10% of 0.5m desired distance
    settled_indices = find(abs(distance_errors) < settling_threshold);
    if ~isempty(settled_indices)
        settling_time = time(settled_indices(1));
    else
        settling_time = t_end;
    end
    
    % Package results
    result.time = time;
    result.robot_x = robot_x_hist;
    result.robot_y = robot_y_hist;
    result.target_x = target_x_hist;
    result.target_y = target_y_hist;
    result.distance_errors = distance_errors;
    result.angle_errors = angle_errors;
    result.avg_dist_error = avg_dist_error;
    result.avg_angle_error = avg_angle_error;
    result.settling_time = settling_time;
end
