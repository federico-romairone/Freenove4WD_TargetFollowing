function [PWM] = speed_to_PWM(speed)
% conversion from speed to PWM

% Settings to import contants from config.py
%   Building the file path from the current directory
targetFolder = fullfile(pwd, '..', 'Python');
% Adding the folder to Python path
insert(py.sys.path, int32(0), targetFolder)
% import config.py module
py.importlib.import_module('config');

a = double(py.config.a);
b = double(py.config.b);
c = double(py.config.c);
d = double(py.config.d);
PWM = a * speed^3 + b * speed^2 + c * speed + d;

end