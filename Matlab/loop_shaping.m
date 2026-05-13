clear
close all  
clc
 
% Laplace variable
s = tf('s');

% Loop transfer functions
Ga = 1; Gf = 1; Gs = 1;
Gp = 1/s;

% Reference requirements
Gc_origin_poles = 1;
kc = 1 * 10^(17/20)

% Transient requirements
s_cap = 0.15;
tr = 1.2;
damping = abs(log(s_cap)) / sqrt(pi^2 + log(s_cap)^2)
Tp = 1 / (2 * damping * sqrt(1 - damping^2))
Sp_num = 2 * damping * sqrt(2 + 4 * damping^2 + 2 * sqrt(1 + 8 * damping^2));
Sp_den = sqrt(1 + 8 * damping^2) + 4 * damping^2 - 1;
Sp = Sp_num / Sp_den
cf_num = (pi - acos(damping)) * sqrt(sqrt(1 + 4 * damping^4) - 2 * damping^2);
cf_den = sqrt(1 - damping^2) * tr;
min_crossover_freq = cf_num / cf_den

% Controller design

Gc = kc/(s^Gc_origin_poles);
L = minreal(zpk(Gc*Gp*Ga*Gf*Gs), 1e-3);

figure(1), bode(L), grid on
[num, den] = tfdata(L, 'v');
figure(2), nyquist1(num, den), grid on
figure(3)
myngridst(Tp, Sp), hold on
nichols(L), hold on

desired_cross_freq = 10;

norm_freq = 1.7;
z = desired_cross_freq / norm_freq
Rz = 1 + s/z

norm_freq = 14.5;
md = 8;
zd = desired_cross_freq / norm_freq
Rd = (1 + s/zd) / (1 + s/(md*zd))

Gc = Gc * Rz * Rd
L = minreal(zpk(Gc*Gp*Ga*Gf*Gs), 1e-3)
nichols(L)

T = minreal(zpk(L/(1+L)));
figure(4), step(T/(Gf*Gs),5), grid on
