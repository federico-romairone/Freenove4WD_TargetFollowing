clc, clear, close all

% Laplace variable
s = tf('s');

% SSR plant matrices
A = 0;
B = 1;
C = -1;
D = 0;

% Initial distance from target (cm)
x0 = 50;

% Reference distance from target (cm)
step_0_pulse_1 = 0;
step_value = 20;
pulse_min = 10;
pulse_max = 20;
pulse_amplitude = pulse_max-pulse_min;
pulse_zero = pulse_min;

% Plant transfer function
plant = ss(A, B, C, D);
Gp = tf(plant);

% Settings to import constants from config.py
targetFolder = fullfile(pwd, '..', 'Python');
insert(py.sys.path, int32(0), targetFolder)

% Force reload to ensure all attributes are visible
mod = py.importlib.import_module('config');
mod = py.importlib.reload(mod);  % <-- riassegna mod con il risultato del reload

% Controller transfer function
Kp = double(mod.Kp);
Gc = Kp;

% Closed-loop transfer function
L = minreal(zpk(Gc * Gp));
S = 1/(1+L);
T = 1-S;

% Values for saturation
max_speed = double(mod.MAX_SPEED);
max_PWM   = double(mod.MAX_PWM);

% Conversion speed to pwm cubic function coefficients
dir_coeff = double(mod.direct_conv_fun_coeff);
% Conversion pwm to speed cubic function coefficients
inv_coeff = double(mod.inverse_conv_fun_coeff);

% Deadzone
balance_deadzone = 1;
dead_zone = double(mod.DEAD_ZONE);

% Run Simulation
out = sim("model_sim.slx");

% Checking internal stability
figure, bode(L), grid on
[num, den] = tfdata(L, 'v');
figure, nyquist1(num, den), grid on
title('Nyquist diagram for loop function')

% Plotting step response
[stepResponse, stepTime] = step(T);
figure
plot(stepTime, stepResponse, 'g'), hold on
plot(stepTime, 0.1 * ones(size(stepTime)), 'r--')
plot(stepTime, 0.9 * ones(size(stepTime)), 'r--')
grid on
title('Closed-loop system step response')
xlabel('Time')
legend('Step response', '10', '90%')

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
