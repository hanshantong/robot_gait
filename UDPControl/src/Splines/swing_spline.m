clear
clc
close all
t_dsp = 0.5;
t_half = 0.5;
ratio = t_dsp / t_half;
tx_sample = [0, 0.01*ratio, 0.02*ratio, 0.15*ratio, 0.35*ratio, 0.45*ratio,...
             0.5*ratio, 0.55, 0.65, ...
             0.75, 0.85, ...
             0.95, 0.99, 1];
x_sample = [0, 0,  0,  0.3, 0.95, ...
            1.00, 1.00, 1.00, ...
            1, 1, ...
            1, 1, 1, 1];
t_lin = linspace(0, 1, 1000);
x_lin = spline(tx_sample, x_sample, t_lin);



tz_sample = [0, 0.01*ratio, 0.1*ratio, 0.13*ratio, 0.22*ratio, 0.35*ratio, ...
    0.45*ratio, 0.6*ratio, 0.99, 1];
z_sample = [0, 0.0, 0.4, 0.6, 0.7, 0.3, 0.15, ...
             0.05,0, 0];
z_lin = spline(tz_sample, z_sample, t_lin);

figure
subplot(3,1,1)
plot(tx_sample, x_sample, 'x')
hold on
grid on
grid minor
plot(t_lin, x_lin, '.')
ylim([-0.1, 1.2])
line([t_dsp t_dsp], [-0.1 1.2])
title('t over x')
subplot(3,1,2)
plot(tz_sample, z_sample, 'x')
hold on
grid on
grid minor
plot(t_lin, z_lin, '.')
ylim([-0.1, 1.2])
line([t_dsp t_dsp], [-0.1 1.2])
title('t over z')
subplot(3,1,3)
plot(x_lin, z_lin, '.')
grid on
grid minor
title('x over z')