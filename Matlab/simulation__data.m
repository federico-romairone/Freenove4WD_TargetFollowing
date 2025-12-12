clc, clear, close all

% Laplace variable
s = tf('s');

% SSR plant matrices
A = 0;
B = 1;
C = 1;
D = 0;

% Initial distance from target (cm)
x0 = 5;

% Reference distance from target (cm)
r = 10;

% Plant transfer function
plant = ss(A, B, C, D);
Gp = tf(plant);

% Settings to import contants from config.py
%   Building the file path from the current directory
targetFolder = fullfile(pwd, '..', 'Python');
%   Adding the folder to Python path
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

%%

% Checking internal stability
figure, bode(L), grid on
[num, den] = tfdata(L, 'v');
figure, nyquist1(num, den), grid on
title('Nyquist diagram for loop function')

% Plotting step response
max_rise_time = 0.5;
[stepResponse, stepTime] = step(T);
figure
plot(stepTime, stepResponse, 'm'), hold on
plot(stepTime, 0.9 * ones(size(stepTime)), 'y--')
plot([max_rise_time max_rise_time], [0 1], 'c--')
grid on
title('Closed-loop system step response')
xlabel('Time')
legend('Step response', '90% of 1', 'Max rise time')

% Plotting results for system response simulation
t = out.y.Time';
r_step = r * ones(1, out.y.Length);
y = out.y.Data';
figure
plot(t, r_step, 'm', t, y, 'b')
grid on
title('System response')
xlabel('Time')
legend('Reference r(t)', 'Output y(t)')
ylim([0 r+0.5])
