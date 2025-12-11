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

%%

% Plotting results for system response simulation
t = out.y.Time';
r_step = r * ones(1, out.y.Length);
r_step_90 = r_step * 0.9;
y = out.y.Data';
figure
plot(t, r_step, 'm', t, r_step_90, 'y--', t, y, 'b')
grid on
title('System response plot')
xlabel('Time')
legend('Reference r(t)', '90% of r(t)', 'Output y(t)')
