clc
clear all

Figure_Settings

addpath('../Bezier-minq');
addpath('../G1fitting');
addpath('../functions');
addpath('export_fig');
addpath('drone_traj');
addpath('GPS_traj');
addpath('../../redebus/ocp-road-sprint-bio-model/data')
addpath('drone_traj')

load trial1_trajectory_AERIAL.mat

%%
geom_apex1  = 60;
geom_apex2  = 163;
% specify road width
road_width      = 14;
semi_road_width = road_width/2;
load redebus_XY
%first_guess_1_5     = [0 180 85 0.0543 0.0535 0.075 -66 -70 0];
%first_guess_6_10    = [0 180 84.5 0.0543 0.0537 0.075 -62 -71 0];
first_guess_1_5     = [0 180 80     0.0543 0.0503 0.075 -66 -70 0];
first_guess_6_10    = [0 180 84.5   0.0543 0.0537 0.075 -62 -71 0];
X = flip(X);
Y = flip(Y);
[d_traj,    theta_traj,     x_traj,     y_traj] = fit_xy(X,Y,1);
d_traj(diff(d_traj)==0)=[];

%% mat files have a structure with time|ibi|sbp inside
files = uigetfile('drone_traj/.mat', 'Select data files', 'MultiSelect', 'on');

if ~iscell(files)
    files = {files};
end %now filename is a cell array regardless of the number of selected files.

figure(1);
hold on
axis equal
fig1. Position = [100 100 800 800];

hold on
% rectangle('Position',[-100, -100, 200, 200], 'FaceColor', [colForestGreen 0.5], 'EdgeColor', 'none',...
%     'LineWidth',3)
% plot(x_traj, y_traj,    'linewidth', 80, 'color', [colSilver 0.1])
text(X(end), Y(end),    ' Finish')
text(X(1), Y(1),        ' Start')
% plot(X, Y, 'o')

% for i = 1:length(X)
%     text(X(i), Y(i), num2str(round(raw_distance(i))), 'fontsize', 4);
% end

plot(x_traj, y_traj, '--', 'linewidth', 2, 'color', colSilver)

plot(x_traj - sin(theta_traj) * semi_road_width, y_traj + cos(theta_traj) * semi_road_width, 'linewidth', 2, 'color', colSlateGray)
plot(x_traj + sin(theta_traj) * semi_road_width, y_traj - cos(theta_traj) * semi_road_width, 'linewidth', 2, 'color', colSlateGray)

for i = 1:length(files)
    % load the files (multiple selection)
    load(files{i});
    if i > 5
        first_guess = first_guess_6_10;
    else
        first_guess = first_guess_1_5;
    end
    
    traj_struct = traj_prop(trajectory, x_traj, y_traj, d_traj, geom_apex1, geom_apex2, first_guess, 'drone');

    plot(traj_struct.x2_int', traj_struct.y2_int', 'linewidth', 3, 'color', [colDodgerBlue 0.5])
end





return

x = -5:0.01:5;

a = 0.8;
b = 1.2;

tol = 0.00001;
eps = 0.00001;

delta = tol * (b - a)/2;
c = -eps / log(cos(pi * (-b + 2 * delta + a) / (-b + a) / 0.2e1));

f = - 10^6 * c * log( cos((pi/2 * (2 * x - (b + a))/(b - a))));

% plot(x,f)
% xlim([a,1])

h   = tol;
H   = 0.8 - h;
A1  = eps/H^2;
A2  = H^2 -eps/(H^2*h^2);
res = 0;

for i = 1:length(x)
    
    if x(i)<= -H
        res(i) = (x(i) +H)^2;
    end
    
    if (x(i)> -H && x(i)< H)
        res(i) = 0;
    end
    
    if x(i)>= H
        res(i) = (x(i) -H)^2;
    end
    
    f2(i) = + A1 * (x(i))^2 - A2 * res(i);
end

close all
figure()
hold on
plot(x, f2)
xlim([-1 1])

