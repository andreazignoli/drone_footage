%%

clc
clear all
close all

%% load midline XY

load redebus_XY.mat
load midline_run_8.mat

%% Orthorectification
% estimate z from distance
distance(1) = 0;
for i = 2: length(X)
    distance(i) = distance_points([X(i); Y(i); Z(i)],[X(i-1); Y(i-1); Z(i-1)]);
end

tot_dist    = cumsum(distance);
F           = griddedInterpolant(tot_dist, Z);

cycled_distance(1) = 0;
for i = 2: length(x0)
    cycled_distance(i) = distance_points([x0(i); y0(i); 0],[x0(i-1); y0(i-1); 0]);
end

tot_cycled_dist = cumsum(cycled_distance);
z0              = F(tot_cycled_dist/tot_cycled_dist(end)*distance(end));
z0              = z0';

%% Optimisation procedure

% clc
% close all
% 
% options = optimset('Display','iter');
% lb      = [-5 175 80 0.05 0.05 -70 -75];
% ub      = [5 185 90 0.06 0.06 -66 -70];
% 
% first_guess = [180 90 0.05 0.05 -66 -70];
% 
% fun     = @(x)find_transformation(x, [X Y], [x0 y0]);
% x_opt   = fmincon(fun, first_guess, [], [], [], [], [], [, [], options);
% 
% gs      = GlobalSearch;
% problem = createOptimProblem('fmincon', 'x0', first_guess,...
%     'objective', fun, 'lb', lb, 'ub', ub);
% 
% x_opt = run(gs, problem);

%% Test the solution

v = [X Y zeros(length(Z), 1)];
u = [x0 y0 zeros(length(Z), 1)];
% example
load trial1_trajectory_AERIAL
t1 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial2_trajectory_AERIAL
t2 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial3_trajectory_AERIAL
t3 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial4_trajectory_AERIAL
t4 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial5_trajectory_AERIAL
t5 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial6_trajectory_AERIAL
t6 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial7_trajectory_AERIAL
t7 = [trajectory(:,1) trajectory(:,2)];
clear trajectory
load trial8_trajectory_AERIAL
t8 = [trajectory(:,1) trajectory(:,2) zeros(length(trajectory(:,1)), 1)];
clear trajectory
load trial9_trajectory_AERIAL
t9 = [trajectory(:,1) trajectory(:,2) zeros(length(trajectory(:,1)), 1)];
clear trajectory
load trial10_trajectory_AERIAL
t10 = [trajectory(:,1) trajectory(:,2) zeros(length(trajectory(:,1)), 1)];
clear trajectory

first_guess_1_5 = [0 180 85 0.0543 0.0535 0.075 -66 -70 0];
first_guess_6_10 = [0 180 84.5 0.0543 0.0537 0.075 -62 -71 0];

% test paraeters

t11     = apply_linear_transformation(t1, first_guess_1_5);
t21     = apply_linear_transformation(t2, first_guess_1_5);
t31     = apply_linear_transformation(t3, first_guess_1_5);
t41     = apply_linear_transformation(t4, first_guess_1_5);
t51     = apply_linear_transformation(t5, first_guess_1_5);
t61     = apply_linear_transformation(t6, first_guess_6_10);
t71     = apply_linear_transformation(t7, first_guess_6_10);
t81     = apply_linear_transformation(t8, first_guess_6_10);
t91     = apply_linear_transformation(t8, first_guess_6_10);
t101    = apply_linear_transformation(t8, first_guess_6_10);
u3      = apply_linear_transformation(u,  first_guess_6_10);

hold on
cla
plot(v(:,1),    v(:,2),     'r')
plot(u3(:,1),   u3(:,2),    'k')
plot(t11(:,1),  t11(:,2),   'k')
plot(t21(:,1),  t21(:,2),   'k')
plot(t31(:,1),  t31(:,2),   'k')
plot(t41(:,1),  t41(:,2),   'k')
plot(t51(:,1),  t51(:,2),   'k')
plot(t61(:,1),  t61(:,2),   'k')
plot(t71(:,1),  t71(:,2),   'k')
plot(t81(:,1),  t81(:,2),   'k')
plot(t91(:,1),  t91(:,2),   'k')
plot(t101(:,1), t101(:,2),  'g')
% legend('Google Earth', 'Drone Midline', 'Run 1', 'Run 2', 'Run 3', 'Run 4', 'location', 'best')
axis equal

return

%% compute linear transformation 3D

u = [x0';y0';z0]; % setting the set of source points
v = [X';Y';Z']; % setting the set of target points
T = findLinearTransformation(u,v); % calculating the transformation

%% test T

I = eye(length(u));
uu = [u;I((3+1):length(u),1:length(u))]; % filling-up the matrix of source points so that you have 5-d points
w = T * uu; % calculating target points
w = w(1:3,1:length(u)); % recovering the 3-d points
w - v % w should match v 

%% so now you try with actual drone footage trajectory

x_AERIAL = trajectory(:,1);
y_AERIAL = trajectory(:,2);

AERIAL_cycled_distance(1) = 0;
for i = 2: length(x_AERIAL)
    AERIAL_cycled_distance(i) = distance_points([x_AERIAL(i); y_AERIAL(i); 0],[x_AERIAL(i-1); y_AERIAL(i-1); 0]);
end

tot_AERIAL_cycled_dist = cumsum(AERIAL_cycled_distance);
z_AERIAL = F(tot_AERIAL_cycled_dist/tot_cycled_dist(end)*distance(end));

%% applying T

u = [x_AERIAL(1:41)'; y_AERIAL(1:41)'; z_AERIAL(1:41)]; % setting the set of source points
I = eye(length(u));
uu = [u;I((3+1):length(u),1:length(u))]; % filling-up the matrix of source points so that you have 5-d points
w = T * uu; % calculating target points
w = w(1:3,1:length(u)); % recovering the 3-d points
w - v % w should match v 

%% 

tform   = fitgeotrans([x0 y0], [X Y], 'similarity');
u3      = [x0 y0 zeros(length(x0),1)] * tform.T;
u3(:,1) = u3(:,1);
u3(:,2) = u3(:,2);

tform2      = fitgeotrans([u3(:,1) u3(:,2)], [X Y], 'nonreflectivesimilarity');
u4          = [u3(:,1) u3(:,2) zeros(length(x0),1)] * tform2.T; 

close all
figure()
hold on
plot(X,Y, 'm')
plot(u3(:,1), u3(:,2), 'ob')
plot(u4(:,1), u4(:,2), 'r')
% plot(u(:,1), u(:,2))
axis equal


%% 

