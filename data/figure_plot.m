close all;clear;clc;

data = readmatrix("waistcar_control_20250805-125523");

% figure(1)
% plot(data(:,1), data(:,5),data(:,1), data(:,8),data(:,1), data(:,6));
% legend("wheel_vel","c_pitch_pos","waist_pos");
% title('c_pitch_pos');
% %xlim([65.55, 100.04]);
% hold on;

figure(1)
plot(data(:,1), data(:,8));
legend("c pitch pos (rad)");
title('c pitch pos');
xlim([30, 90]);
hold on;

% figure(2)
% plot(data(:,1), data(:,11), 'b-', 'LineWidth', 1.5);
% hold on;
% yline(-0.05, 'k--', 'LineWidth', 1.2); 
% hold off;
% title('c pitch pos');
% %xlim([144.4, 148.1]);
% 
% figure(3)
% plot(data(:,1), data(:,7)* 0.09, 'b-', 'LineWidth', 1.5);
% title('wheel vel');
% %xlim([144.4, 148.1]);
% 
% figure(4)
% plot(data(:,1), data(:,14), 'b-', 'LineWidth', 1.5);
% title('wheel torque');
% %xlim([79, 140]);
% 
% figure(5)
% plot(data(:,1), data(:,14), 'b-', 'LineWidth', 1.5);
% title('waist torque');
% %xlim([17.7, 18.1]);
% 
% figure(6)
% plot(data(:,1), data(:,6), 'b-', 'LineWidth', 1.5);
% title('waist pos');
% %xlim([79, 140]);
% 
% figure(7)
% plot(data(:,1), data(:,8), 'b-', 'LineWidth', 1.5);
% title('waist vel');
% %xlim([17.7, 18.1]);
% 
% 
% 
% 
