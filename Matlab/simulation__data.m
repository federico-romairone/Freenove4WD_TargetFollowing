clear
close all
clc

% Laplace variable
s = tf('s');

% Initial distance from target (cm)
x0 = 5;

% Reference distance from target (cm)
step_0_pulse_1 = 0;
step_value = 20;
pulse_min = 10;
pulse_max = 20;
pulse_amplitude = pulse_max-pulse_min;
pulse_zero = pulse_min;

% Plant transfer function
Gp = 1/s;

% Settings to import constants from config.py
targetFolder = fullfile(pwd, '..', 'Python');
insert(py.sys.path, int32(0), targetFolder)

% Force reload to ensure all attributes are visible
mod = py.importlib.import_module('config');
mod = py.importlib.reload(mod);

% Deadzone for convertion funtion
dead_zone_width = double(mod.DEAD_ZONE_WIDTH);

% Controller transfer function from loop_shaping.m
kc = 0.4 * 7.0795;
Gc = kc/s * (1 + s/5.882) * ((5.517*s + 3.805)/(0.6897*s + 3.805))

% Descrete controller
T_sampling = double(mod.SAMPLING_PERIOD);
Gc_d = c2d(Gc, T_sampling, 'matched')
[NG_coeff, DG_coeff] = tfdata(Gc_d, 'v');

% Values for saturation
max_speed = double(mod.MAX_SPEED);
max_PWM   = double(mod.MAX_PWM);

% Conversion speed to pwm cubic function coefficients
dir_coeff = double(mod.direct_conv_fun_coeff);
% Conversion pwm to speed cubic function coefficients
inv_coeff = double(mod.inverse_conv_fun_coeff);

% Run Simulation
out = sim("model_sim.slx");

% Plotting results for system response simulation
t = out.output.Time';
y = out.output.Data';
r = out.reference.Data';
e = out.error.Data';
figure
plot(t, y, 'g', t, r, 'r--', t, e, 'b')
grid on
title('System response and error')
xlabel('Time')
legend('Output y(t)', 'Reference r(t)', 'Error e(t)')
