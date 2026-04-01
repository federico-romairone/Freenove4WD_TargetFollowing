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

% Settings to import contants from config.py
%   Building the file path from the current directory
targetFolder = fullfile(pwd, '..', 'Python');
% Adding the folder to Python path
insert(py.sys.path, int32(0), targetFolder)
% import config.py module
py.importlib.import_module('config');

% Controller transfer function 
Kp = double(py.config.Kp);
Gc = Kp;

% Closed-loop transfer function
L = minreal(zpk(Gc * Gp));
S = 1/(1+L);
T = 1-S;

% Values for saturation
max_speed = double(py.config.MAX_SPEED);
max_PWM = double(py.config.MAX_PWM);

% convertion speed to pwm cubic function coefficients
a = double(py.config.a);
b = double(py.config.b);
c = double(py.config.c);
d = double(py.config.d);
% convertion pwm to speed logaritmic function coefficients
a_log = double(py.config.a_log);
b_log = double(py.config.b_log);

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
