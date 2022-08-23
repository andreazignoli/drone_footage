clc
clear all
close all

Figure_Settings

addpath('../Bezier-minq');
addpath('../G1fitting');
addpath('../functions');
addpath('export_fig');
addpath('drone_traj');
addpath('GPS_traj');
addpath('../../redebus/ocp-road-sprint-bio-model/data')

% %% load midline course
%
% % load the file you exported from GPS visualiser
% [FileName,PathName] = uigetfile('../GPS/*.txt','Select the data file with altitude (GPS visualizer)');
% disp('http://www.gpsvisualizer.com/convert?output_elevation');
%
%
% % read the file
% fileID = fopen([PathName, FileName]);
%
% nline = 0;
% tline = fgetl(fileID);
% while ischar(tline)
%     tline = fgetl(fileID);
%     nline = nline+1;
% end
%
% % extraction
% kmlwithaltitude = importfilewithaltitude([PathName, FileName]);
%
% lat     = kmlwithaltitude.latitude;
% long    = kmlwithaltitude.longitude;
% alt     = kmlwithaltitude.altitudem;
%
% %% Define Point 0
%
% % NOTE: Google Earth uses the WGS84 projection whereas Google Maps uses a
% % close variant of the Mercator projection.
%
% % lat0    = 46.155859; % gmaps
% % long0   = 11.309244; % gmaps
% % lat0    = 46.155894; % gstreet ?
% % long0   = 11.309133; % gstreet ?
% % lat0    = 46.155856; % gearth POINT 0
% % long0   = 11.309244; % gearth POINT 0
% % lat1    = 46.155794; % gearth POINT 1
% % long1   = 11.307461; % gearth POINT 1
% % alt0    = 0;
% % course
%
% [X,Y,Z] = geodetic2enu(lat,long,alt,lat(1),long(1),alt(1),wgs84Ellipsoid);
%
% xp0 = 0;
% yp0 = 0;
% zp0 = 0;

%% PROCESSING the course

% specify road width
road_width      = 14;
semi_road_width = road_width/2;
load redebus_XY
X = flip(X);
Y = flip(Y);
raw_distance = [0];
for i = 2:length(X)
    raw_distance = [raw_distance; distance_points([X(i), Y(i), 0],[X(i-1), Y(i-1), 0])];
end
raw_distance = cumsum(raw_distance);

[d_traj,    theta_traj,     x_traj,     y_traj] = fit_xy(X,Y,1);

x_traj(diff(d_traj)==0)     = [];
y_traj(diff(d_traj)==0)     = [];
theta_traj(diff(d_traj)==0) = [];
d_traj(diff(d_traj)==0)     = [];

% traj int
d_traj_int      = interp1(d_traj,d_traj,d_traj:0.5:d_traj(end));
x_traj_int      = interp1(d_traj,x_traj,d_traj:0.5:d_traj(end));
y_traj_int      = interp1(d_traj,y_traj,d_traj:0.5:d_traj(end));
d_traj_int      = interp1(d_traj,d_traj,d_traj:0.5:d_traj(end));
theta_traj_int  = interp1(d_traj,theta_traj,d_traj:0.5:d_traj(end), 'PCHIP');

% geometric apex
tmp_geo     = findchangepts(theta_traj_int, 'Statistic','linear','MaxNumChanges',4);
% geom_apex1  = round(mean(tmp_geo(1:2)));
% geom_apex2  = round(mean(tmp_geo(3:4)));
geom_apex1  = 60;
geom_apex2  = 163;

%% load XY trajectory either from the GPS or the motion capture
% (please notice that Tc will change)

first_guess_1_5     = [0 180 85 0.0543 0.0535 0.075 -66 -70 0];
first_guess_6_10    = [0 180 84.5 0.0543 0.0537 0.075 -62 -71 0];

% 1-> GPS
% load trajectory_example_GPS
load trajectory_example_GPS_trial_1
% 2-> AERIAL

% load trajectory_example_AERIAL
% 3-> PINS (optimal trajectory)
% load traj_PINS

%% to do for every drone shot

fig1 = figure(1);
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
% plot(x_traj1, y_traj1,  'color', colOrangeRed)
axis equal ;

%%
fig2            = figure();
fig2.Position   = [100 100 1200 600];
subplot(3,1,3)
hold on
theta_road_grid = griddedInterpolant(d_traj_int, theta_traj_int - pi);
% plot(d_traj_int, theta_traj_int - pi, 'k-', 'linewidth', 2)

% fig3            = figure();
% fig3.Position   = [100 100 600 600];
% hold on
% ellipse(1, 1, 0, 0, 0, [.9 .9 .9]);
% ellipse(.5, .5,0,0, 0, [.9 .9 .9]);

%% PINS

[txt_file,txt_file_path] = uigetfile('../../redebus/ocp-road-sprint-bio-model/data/.txt', 'MultiSelect', 'on');

if ~iscell(txt_file)
    txt_file = {txt_file};
end %now filename is a cell array regardless of the number of selected files.

XX_pins                  = [];
YY_pins                  = [];
vv_pins                  = [];
aa_pins                  = [];
nn_pins                  = [];
tt_pins                  = [];
ss_pins                  = [];
total_travelled_pins     = [];
final_time_pins          = [];
apex_1_pins              = [];
apex_2_pins              = [];
braking_distance1_pins   = [];
braking_distance2_pins   = [];
turn_in_distance1_pins   = [];
turn_in_distance2_pins   = [];

for i = 1:length(txt_file)
    disp(txt_file{i})
    % 
    D                           = importdata(txt_file{i});
    xCoM                        = getData(D,'xCoM');
    yCoM                        = getData(D,'yCoM');
    tmp                         = getData(D,'t');
    zeta                        = getData(D,'zeta');
    power                       = getData(D,'Power');
    alpha                       = getData(D,'alpha');
    vel                         = getData(D,'u');
    ay                          = getData(D,'ay');
    ax                          = getData(D,'ax');
    n                           = getData(D,'n');
    theta                       = getData(D,'theta');
    xLeftEdge                   = getData(D,'xLeftEdge');
    yLeftEdge                   = getData(D,'yLeftEdge');
    xRightEdge                  = getData(D,'xRightEdge');
    yRightEdge                  = getData(D,'yRightEdge');
    
    % struct
    pins_struct = traj_prop([xCoM yCoM], x_traj, y_traj, d_traj, geom_apex1, geom_apex2, first_guess_1_5, 'PINS');
    
    % create time grid for velocity calculations
    s_grid      = griddedInterpolant(make_robust(zeta), zeta);
    a_grid      = griddedInterpolant(make_robust(zeta), alpha);
    n_grid      = griddedInterpolant(make_robust(zeta), - n);
    v_grid      = griddedInterpolant(make_robust(zeta), vel);
    xx_grid     = griddedInterpolant(make_robust(zeta), xCoM);
    yy_grid     = griddedInterpolant(make_robust(zeta), yCoM);
    ay_grid     = griddedInterpolant(make_robust(zeta), ay);
    ax_grid     = griddedInterpolant(make_robust(zeta), ax);
    tt_grid     = griddedInterpolant(make_robust(zeta), theta + alpha);
    
    % compute velocity
    [d_tot, v_tot] = compute_v(pins_struct.x2_int, pins_struct.y2_int, diff(tmp));
    % 
    XX_pins                     = [XX_pins xx_grid(pins_struct.s2_int(1:427))'];
    YY_pins                     = [YY_pins yy_grid(pins_struct.s2_int(1:427))'];
    ss_pins                     = [ss_pins s_grid(pins_struct.s2_int(1:427))'];
    aa_pins                     = [aa_pins optimal_filter(pins_struct.s2_int(1:427), ... 
                                                          a_grid(pins_struct.s2_int(1:427)), ...
                                                          20)];
    vv_pins                     = [vv_pins v_grid(pins_struct.s2_int(1:427))'];
    nn_pins                     = [nn_pins n_grid(pins_struct.s2_int(1:427))'];
    tt_pins                     = [tt_pins optimal_filter(pins_struct.s2_int(1:427), ... 
                                                          tt_grid(pins_struct.s2_int(1:427)), ...
                                                          20)];
    final_time_pins             = [final_time_pins;         tmp(end)];
    total_travelled_pins        = [total_travelled_pins;    d_tot(end)];
    
    % compute with speed data not from positions
    % take the first and the fourth
%     [turn_in_tmp_1, turn_in_tmp_idx_1] = max(optimal_filter(pins_struct.s2_int(1:200), ... 
%                                                           a_grid(pins_struct.s2_int(1:200)), ...
%                                                           20));
%     [turn_in_tmp_2, turn_in_tmp_idx_2] = min(optimal_filter(pins_struct.s2_int(200:end), ... 
%                                                           a_grid(pins_struct.s2_int(200:end)), ...
%                                                           20));
    
    turn_in_tmp_tmp = findchangepts(n_grid(pins_struct.s2_int(1:427))', 'Statistic','linear','MaxNumChanges', 5); 
                                                      
    % take the first and the fourth
    braking_points_tmp = findchangepts(v_grid(pins_struct.s2_int(1:427))', 'Statistic','linear','MaxNumChanges', 5); 
    
    % apex
    [apex_tmp_1, apex_tmp_idx_1] = min(n_grid(pins_struct.s2_int(1:200)));
    [apex_tmp_2, apex_tmp_idx_2] = max(n_grid(pins_struct.s2_int(200:end)));
    
    apex_1_pins                 = [apex_1_pins;             pins_struct.s2_int(apex_tmp_idx_1) - geom_apex1];
    apex_2_pins                 = [apex_2_pins;             pins_struct.s2_int(apex_tmp_idx_2 + 200) - geom_apex2];
    braking_distance1_pins      = [braking_distance1_pins;  geom_apex1 - pins_struct.s2_int(braking_points_tmp(1))];
    braking_distance2_pins      = [braking_distance2_pins;  geom_apex2 - pins_struct.s2_int(braking_points_tmp(4))];
    turn_in_distance1_pins      = [turn_in_distance1_pins;  geom_apex1 - pins_struct.s2_int(turn_in_tmp_tmp(1))];
    turn_in_distance2_pins      = [turn_in_distance2_pins;  geom_apex2 - pins_struct.s2_int(turn_in_tmp_tmp(4))];
    
    figure(fig1);
    hold on
    plot(pins_struct.x2_int, pins_struct.y2_int, 'linewidth', 3, 'color', [colOrangeRed 0.5])
    % plot apex
%     scatter(pins_struct.x2_int(pins_struct.apex1_idx2), pins_struct.y2_int(pins_struct.apex1_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
%     scatter(pins_struct.x2_int(pins_struct.apex2_idx2), pins_struct.y2_int(pins_struct.apex2_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
%     
%     % turn-in points
%     scatter(pins_struct.x2_int(turn_in_tmp_tmp(1)), pins_struct.y2_int(turn_in_tmp_tmp(1)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
%     scatter(pins_struct.x2_int(turn_in_tmp_tmp(4)), pins_struct.y2_int(turn_in_tmp_tmp(4)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
%     scatter(pins_struct.x2_int(braking_points_tmp(1)), pins_struct.y2_int(braking_points_tmp(1)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
%     scatter(pins_struct.x2_int(braking_points_tmp(4)), pins_struct.y2_int(braking_points_tmp(4)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
    
%     figure(fig3);
%     plot(ay_grid(pins_struct.s2_int)/9.8, ax_grid(pins_struct.s2_int)/9.8, 'color', colOrangeRed)
%     ylabel('Long acc (m/s^2)')
%     xlabel('Lat acc (m/s^2)')
%     ylim([-1,1])
%     xlim([-1,1])
%     axis equal
%     box on
%     grid minor
    
end

figure(fig2);
%%
subplot(3,1,1)
hold on
xline(- mean(braking_distance1_pins) + geom_apex1, 'linewidth', 2, 'color', colRed, 'linestyle', '-')
xline(- mean(braking_distance1_pins) + geom_apex1 + std(braking_distance1_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(- mean(braking_distance1_pins) + geom_apex1 - std(braking_distance1_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(- mean(braking_distance2_pins) + geom_apex2, 'linewidth', 2, 'color', colRed, 'linestyle', '-')
xline(- mean(braking_distance2_pins) + geom_apex2 + std(braking_distance2_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(- mean(braking_distance2_pins) + geom_apex2 - std(braking_distance2_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
fill_between(mean(ss_pins,2), ...
            (mean(vv_pins,2) + std(vv_pins, 0, 2)) * 3.6, ...
            (mean(vv_pins,2) - std(vv_pins, 0, 2)) * 3.6, [], colOrangeRed)
plot(mean(ss_pins,2), mean(vv_pins, 2) * 3.6, 'color', colOrangeRed)
xline(geom_apex1, 'linewidth', 2, 'color', colSlateGray, 'linestyle', ':')
xline(geom_apex2, 'linewidth', 2, 'color', colSlateGray, 'linestyle', ':')
ax1 = gca;
ax1.XTick = [];
box on
grid minor
xlim([0, 213])
subplot(3,1,2)
hold on
xline(geom_apex1, 'linewidth', 2, 'color', colSlateGray, 'linestyle', ':')
xline(geom_apex2, 'linewidth', 2, 'color', colSlateGray, 'linestyle', ':')
xline(mean(apex_1_pins) + geom_apex1, 'linewidth', 2, 'color', colRed, 'linestyle', '-')
xline(mean(apex_1_pins) + geom_apex1 + std(apex_1_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(mean(apex_1_pins) + geom_apex1 - std(apex_1_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(mean(apex_2_pins) + geom_apex2, 'linewidth', 2, 'color', colRed, 'linestyle', '-')
xline(mean(apex_2_pins) + geom_apex2 + std(apex_2_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(mean(apex_2_pins) + geom_apex2 - std(apex_2_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
fill_between(mean(ss_pins,2), ...
            (mean(nn_pins,2) + std(nn_pins, 0, 2)), ...
            (mean(nn_pins,2) - std(nn_pins, 0, 2)), [], colOrangeRed)
plot(mean(ss_pins,2), mean(nn_pins, 2), 'color', colOrangeRed)
ylim([-8,8])
ax2 = gca;
ax2.XTick = [];
box on
grid minor
xlim([0, 213])
subplot(3,1,3)
hold on
xline(geom_apex1, 'linewidth', 2, 'color', colSlateGray, 'linestyle', ':')
xline(geom_apex2, 'linewidth', 2, 'color', colSlateGray, 'linestyle', ':')
xline(- mean(turn_in_distance1_pins) + geom_apex1, 'linewidth', 2, 'color', colRed, 'linestyle', '-')
xline(- mean(turn_in_distance1_pins) + geom_apex1 + std(turn_in_distance1_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(- mean(turn_in_distance1_pins) + geom_apex1 - std(turn_in_distance1_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(- mean(turn_in_distance2_pins) + geom_apex2, 'linewidth', 2, 'color', colRed, 'linestyle', '-')
xline(- mean(turn_in_distance2_pins) + geom_apex2 + std(turn_in_distance2_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
xline(- mean(turn_in_distance2_pins) + geom_apex2 - std(turn_in_distance2_pins), 'linewidth', 0.5, 'color', colRed, 'linestyle', '--')
fill_between(mean(ss_pins,2), ...
            (mean(aa_pins,2) + std(aa_pins, 0, 2)), ...
            (mean(aa_pins,2) - std(aa_pins, 0, 2)), [], colOrangeRed)
plot(mean(ss_pins,2), mean(aa_pins, 2), 'color', colOrangeRed)
ax3 = gca;
% ax3.XTick = [];
box on
grid minor
xlim([0, 213])
breakpoint = true;

%% DRONE SHOTS
% mat files have a structure with time|ibi|sbp inside
files = uigetfile('drone_traj/.mat', 'Select data files', 'MultiSelect', 'on');

if ~iscell(files)
    files = {files};
end %now filename is a cell array regardless of the number of selected files.

XX                  = [];
YY                  = [];
vv                  = [];
aa                  = [];
nn                  = [];
tt                  = [];
ss                  = [];
final_time          = [];
total_travelled     = [];
apex_1              = [];
apex_2              = [];
braking_distance1   = [];
braking_distance2   = [];
turn_in_distance1   = [];
turn_in_distance2   = [];

for i = 1:length(files)
    % load the files (multiple selection)
    load(files{i});
    if i > 5
        first_guess = first_guess_6_10;
    else
        first_guess = first_guess_1_5;
    end
    traj_struct = traj_prop(trajectory, x_traj, y_traj, d_traj, geom_apex1, geom_apex2, first_guess, 'drone');
    [d_tot, ~]  = compute_v(traj_struct.x2_int,traj_struct.y2_int,1);
    
    % plot(traj_struct.x2_int, traj_struct.y2_int, 'Color', [colDodgerBlue 0.2], 'linewidth', 2)
    %     surface_line = surface( 'XData', [traj_struct.x2_int' traj_struct.x2_int'], ... % N.B.  XYZC Data must have at least 2 cols
    %                             'YData', [traj_struct.y2_int' traj_struct.y2_int'], ...
    %                             'ZData', zeros(numel(traj_struct.y2_int),2), ...
    %                             'CData', [traj_struct.v2_F traj_struct.v2_F], ...
    %                             'FaceColor', 'none', ...
    %                             'EdgeColor', 'interp', ...
    %                             'Marker', 'none', ...
    %                             'linewidth', 0.5);
    % hold on
    
    XX          = [XX traj_struct.x2_int'];
    YY          = [YY traj_struct.y2_int'];
    aa          = [aa traj_struct.alpha2_F - optimal_filter(traj_struct.s2_int, ...
                                                            theta_road_grid(traj_struct.s2_int)', 20)];
    vv          = [vv traj_struct.v2_F];
    nn          = [nn traj_struct.n2_int'];
    tt          = [tt traj_struct.theta2_int'];
    ss          = [ss traj_struct.s2_int'];
    
    a_grid_drone = griddedInterpolant(traj_struct.s2_int, traj_struct.alpha2_F - optimal_filter(traj_struct.s2_int, theta_road_grid(traj_struct.s2_int)', 20));
    
%     [turn_in_tmp_1, turn_in_tmp_idx_1] = max(optimal_filter(traj_struct.s2_int(1:200), ... 
%                                                           a_grid_drone(traj_struct.s2_int(1:200)), ...
%                                                           20));
%     [turn_in_tmp_2, turn_in_tmp_idx_2] = min(optimal_filter(traj_struct.s2_int(200:end), ... 
%                                                           a_grid_drone(traj_struct.s2_int(200:end)), ...
%                                                           20));
                                                      
	turn_in_tmp_tmp = findchangepts(traj_struct.n2_int, 'Statistic','linear','MaxNumChanges', 6); 
    
    final_time              = [final_time; d_tot./mean(traj_struct.v2_F)];
    total_travelled         = [total_travelled; d_tot(end)]; 
                                                      
    % take the first and the fourth
    braking_points_tmp = findchangepts(traj_struct.v2_int, 'Statistic','linear','MaxNumChanges', 5); 
    
    % apex
    [apex_tmp_1, apex_tmp_idx_1] = min(traj_struct.n2_int(1:200));
    [apex_tmp_2, apex_tmp_idx_2] = max(traj_struct.n2_int(200:end));
    
    apex_1                 = [apex_1;             traj_struct.s2_int(apex_tmp_idx_1) - geom_apex1];
    apex_2                 = [apex_2;             traj_struct.s2_int(apex_tmp_idx_2 + 200) - geom_apex2];
    braking_distance1      = [braking_distance1;  geom_apex1 - traj_struct.s2_int(braking_points_tmp(1))];
    braking_distance2      = [braking_distance2;  geom_apex2 - traj_struct.s2_int(braking_points_tmp(4))];
    turn_in_distance1      = [turn_in_distance1;  geom_apex1 - traj_struct.s2_int(turn_in_tmp_tmp(1))];
    turn_in_distance2      = [turn_in_distance2;  geom_apex2 - traj_struct.s2_int(turn_in_tmp_tmp(4))];
    
    figure(fig1);
    hold on
    plot(traj_struct.x2_int', traj_struct.y2_int', 'linewidth', 3, 'color', [colDodgerBlue 0.5])
    % plot apex
%     scatter(traj_struct.x2_int(apex_tmp_idx_1), traj_struct.y2_int(apex_tmp_idx_1), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
%     scatter(traj_struct.x2_int(apex_tmp_idx_2 + 200), traj_struct.y2_int(apex_tmp_idx_2 + 200), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
%     
%     % turn-in points
%     scatter(traj_struct.x2_int(turn_in_tmp_tmp(1)), traj_struct.y2_int(turn_in_tmp_tmp(1)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
%     scatter(traj_struct.x2_int(turn_in_tmp_tmp(4)), traj_struct.y2_int(turn_in_tmp_tmp(4)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
%     scatter(traj_struct.x2_int(braking_points_tmp(1)), traj_struct.y2_int(braking_points_tmp(1)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
%     scatter(traj_struct.x2_int(braking_points_tmp(4)), traj_struct.y2_int(braking_points_tmp(4)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
    
end

figure(fig2);
%% 
subplot(3,1,1)
title('Speed and braking points')
hold on
xline(- mean(braking_distance1) + geom_apex1, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
xline(- mean(braking_distance1) + geom_apex1 + std(braking_distance1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline(- mean(braking_distance1) + geom_apex1 - std(braking_distance1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline(- mean(braking_distance1) + geom_apex2, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
xline(- mean(braking_distance1) + geom_apex2 + std(braking_distance2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline(- mean(braking_distance1) + geom_apex2 - std(braking_distance2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
fill_between(mean(ss,2), ...
            (mean(vv,2) + std(vv, 0, 2)) * 3.6, ...
            (mean(vv,2) - std(vv, 0, 2)) * 3.6, [], colDodgerBlue)
plot(mean(ss,2), mean(vv, 2) * 3.6, 'color', colDodgerBlue)
ax1 = gca;
ax1.XTick = [];
ylabel('Speed (km/h)')
box on
grid minor
xlim([0, 213])
subplot(3,1,2)
title('Lateral displacement and apexes')
hold on
% xline(- mean(apex_1) + geom_apex1, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
% xline(- mean(apex_1) + geom_apex1 + std(apex_1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
% xline(- mean(apex_1) + geom_apex1 - std(apex_1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
% xline(- mean(apex_2) + geom_apex2, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
% xline(- mean(apex_2) + geom_apex2 + std(apex_2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
% xline(- mean(apex_2) + geom_apex2 - std(apex_2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline( mean(apex_1) + geom_apex1, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
xline( mean(apex_1) + geom_apex1 + std(apex_1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline( mean(apex_1) + geom_apex1 - std(apex_1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline( mean(apex_2) + geom_apex2, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
xline( mean(apex_2) + geom_apex2 + std(apex_2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline( mean(apex_2) + geom_apex2 - std(apex_2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
fill_between(mean(ss,2), ...
            (mean(nn,2) + std(nn, 0, 2)), ...
            (mean(nn,2) - std(nn, 0, 2)), [], colDodgerBlue)
plot(mean(ss,2), mean(nn, 2), 'color', colDodgerBlue)
ylim([-8, 8])
ax2 = gca;
ax2.XTick = [];
box on
grid minor
ylabel('Lat dist (m)')
xlim([0, 213])
subplot(3,1,3)
title('Heading and turning points')
hold on
xline(- mean(turn_in_distance1) + geom_apex1, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
xline(- mean(turn_in_distance1) + geom_apex1 + std(turn_in_distance1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline(- mean(turn_in_distance1) + geom_apex1 - std(turn_in_distance1), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline(- mean(turn_in_distance2) + geom_apex2, 'linewidth', 2, 'color', colDarkBlue, 'linestyle', '-')
xline(- mean(turn_in_distance2) + geom_apex2 + std(turn_in_distance2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
xline(- mean(turn_in_distance2) + geom_apex2 - std(turn_in_distance2), 'linewidth', 0.5, 'color', colDarkBlue, 'linestyle', '--')
fill_between(mean(ss,2), ...
            (mean(aa,2) + std(aa, 0, 2)) - pi, ...
            (mean(aa,2) - std(aa, 0, 2)) - pi, [], colDodgerBlue)
plot(mean(ss,2), mean(aa, 2) - pi, 'color', colDodgerBlue)
ax3 = gca;
% ax3.XTick = [];
box on
grid minor
ylabel('Heading (rad)')
xlim([0, 213])
    
return


% colormap hot;
% colorbar;
% surface_line = surface( 'XData',        [xCoM xCoM], ... % N.B.  XYZC Data must have at least 2 cols
%                             'YData',    [yCoM yCoM], ...
%                             'ZData',    zeros(numel(yCoM),2), ...
%                             'CData',    [vel * 3.6 vel * 3.6], ...
%                             'FaceColor',    'none', ...
%                             'EdgeColor',    'interp', ...
%                             'Marker',       'none', ...
%                             'linewidth',    4);

% plot apex
scatter(pins_struct.x2_int(pins_struct.apex1_idx2), pins_struct.y2_int(pins_struct.apex1_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(pins_struct.x2_int(pins_struct.apex2_idx2), pins_struct.y2_int(pins_struct.apex2_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')

% turn-in points
scatter(pins_struct.x2_int(pins_struct.turn_in_points2(1:2:end)), pins_struct.y2_int(pins_struct.turn_in_points2(1:2:end)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
scatter(pins_struct.x2_int(pins_struct.braking_points2(1:2:end)), pins_struct.y2_int(pins_struct.braking_points2(1:2:end)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')

scatter(X(1), Y(1), 20)
scatter(X(1), Y(1), 60)

scatter(X(end), Y(end), 20)
scatter(X(end), Y(end), 60)
scatter(X, Y, 20, colSlateGray,'filled','MarkerFaceAlpha',.2,'MarkerEdgeAlpha',0)
% plot(x_traj2, y_traj2, 'k--')
% grid minor
xlabel('x (m)')
ylabel('y (m)')
axis equal ;
axis tight ;

xlim([-80 20])
ylim([-60 70])
box off
grid minor

% export_fig('Trajectories', '-dpng', '-transparent', '-r300');
export_fig('Trajectories', '-dpng', '-r300');

return

% apex
% scatter(x1_int(apex1_idx), y1_int(apex1_idx), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(x2_int(apex1_idx2), y2_int(apex1_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
% text(x1_int(apex1_idx), y1_int(apex1_idx), '  Apex 1', 'FontSize', 10)

% scatter(x1_int(apex2_idx), y1_int(apex2_idx), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(x2_int(apex2_idx2), y2_int(apex2_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
% text(x1_int(apex2_idx), y1_int(apex2_idx), '  Apex 2', 'FontSize', 10)

% geom apex
% plot(   [x_traj_int(geom_apex1) - sin(theta_traj_int(geom_apex1)) * semi_road_width,   x_traj_int(geom_apex1) + sin(theta_traj_int(geom_apex1)) * semi_road_width],...
%         [y_traj_int(geom_apex1) + cos(theta_traj_int(geom_apex1)) * semi_road_width,   y_traj_int(geom_apex1) - cos(theta_traj_int(geom_apex1)) * semi_road_width], 'color', colSlateGray, 'linewidth', 4)
plot(   [x_traj_int(geom_apex2) - sin(theta_traj_int(geom_apex2)) * semi_road_width,   x_traj_int(geom_apex2) + sin(theta_traj_int(geom_apex2)) * semi_road_width],...
    [y_traj_int(geom_apex2) + cos(theta_traj_int(geom_apex2)) * semi_road_width,   y_traj_int(geom_apex2) - cos(theta_traj_int(geom_apex2)) * semi_road_width], 'color', colSlateGray, 'linewidth', 4)

% turn-in point
% scatter(x1_int(turn_in_points1(1:2:end)), y1_int(turn_in_points1(1:2:end)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
scatter(x2_int(turn_in_points2(1:2:end)), y2_int(turn_in_points2(1:2:end)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
% text(x1_int(turn_in_points1(1:2:end)), y1_int(turn_in_points1(1:2:end)), '  Turn-in point', 'FontSize', 10)
% braking point
% scatter(x1_int(braking_points1(1:2:end)), y1_int(braking_points1(1:2:end)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
scatter(x2_int(braking_points2(1:2:end)), y2_int(braking_points2(1:2:end)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
% text(x1_int(braking_points1(1:2:end)), y1_int(braking_points1(1:2:end)), '  Braking point', 'FontSize', 10)

return


[d_traj1, theta_traj1, x_traj1, y_traj1] = fit_xy(XGPS,YGPS,1);
[d1, v1]    = compute_v(XGPS,YGPS,1);

% compute curvilinear abscissa and lateral displacement
% [s1, n1, theta1, x1, y1] = compute_n(x_traj1,y_traj1,x_traj,y_traj,d_traj);


%% PROCESSING the trajectory: interpolating to a common curvilinear abscissa

% coordinates interp
% x1_int      = interp1(s1,x1,d_traj_int(d_traj_int<d1(end)), 'PCHIP');
% y1_int      = interp1(s1,y1,d_traj_int(d_traj_int<d1(end)), 'PCHIP');


% lateral diplacement
% n1_int      = interp1(s1,n1,d_traj_int(d_traj_int<d1(end)), 'PCHIP');


% % alpha
% alpha1_int  = interp1(s1,theta1,d_traj_int(d_traj_int<d1(end)), 'PCHIP');
% alpha1_F    = optimal_filter(d_traj_int(d_traj_int<d1(end)), alpha1_int, 20);
%
%
% % speed
% v1_int      = interp1(d1,v1,d_traj_int(d_traj_int<d1(end)), 'PCHIP');
% v1_F        = optimal_filter(d_traj_int(d_traj_int<d1(end)), v1_int, 20);
%
%
% % trajectory crucial points
% turn_in_points1 = findchangepts(alpha1_F,   'Statistic','linear','MaxNumChanges',4);
% braking_points1 = findchangepts(v1_F,       'Statistic','linear','MaxNumChanges',4);
%
%
% % apex first hairpin
% [apex1_val, apex1_idx] = min(n1_int(d_traj_int<100));
%
% % apex second hairpin
% [apex2_val, apex2_idx]      = max(n1_int(d_traj_int>100 & d_traj_int<d1(end)));
% apex2_idx                   = apex2_idx + length(d_traj_int( 1:find( d_traj_int> 100, 1 ) ));
%
%
% %% hairpin 1
% % distance between geom apex and turn-in point (on s!)
% turn_in_distance1    = d_traj_int(geom_apex1)    - d_traj_int(turn_in_points1(1));
% braking_distance1    = d_traj_int(geom_apex1)    - d_traj_int(braking_points1(1));
% apex_distance1       = d_traj_int(apex1_idx)     - d_traj_int(geom_apex1) ;
%
%
% %% hairpin 2
% % distance between geom apex and turn-in point (on s!)
% turn_in_distance2    = d_traj_int(geom_apex2)    - d_traj_int(turn_in_points1(3));
% braking_distance2    = d_traj_int(geom_apex2)    - d_traj_int(braking_points1(3));
% apex_distance2       = d_traj_int(apex2_idx)     - d_traj_int(geom_apex2) ;
%
% %% comparison between trajectories is done until the lates common s value
% min_length_idx  = min(length(n1_int), length(n2_int));
% MAE_n           = mean(abs(n1_int(1:min_length_idx)     - n2_int(1:min_length_idx)));
% MAE_alpha       = mean(abs(alpha1_F(1:min_length_idx)   - alpha2_F(1:min_length_idx)))  * 180/pi;
% MAE_v           = mean(abs(v1_F(1:min_length_idx)       - v2_F(1:min_length_idx)))      * 3.6;

%% post process plot

fig = figure();
fig. Position = [100 100 800 800];

hold on
text(X(end), Y(end),    ' Finish')
text(X(1), Y(1),        ' Start')
plot(X, Y, 'o')
plot(x_traj, y_traj,    '--', 'linewidth', 2, 'color', colSilver)
plot(x_traj - sin(theta_traj) * semi_road_width, y_traj + cos(theta_traj) * semi_road_width, 'linewidth', 2, 'color', colSlateGray)
plot(x_traj + sin(theta_traj) * semi_road_width, y_traj - cos(theta_traj) * semi_road_width, 'linewidth', 2, 'color', colSlateGray)
% plot(x_traj1, y_traj1,  'color', colOrangeRed)
plot(x_traj2, y_traj2,  'color', colCornFlowerBlue)
plot(xCoM, yCoM,        'color', colOrangeRed)

% apex
% scatter(x1_int(apex1_idx), y1_int(apex1_idx), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(x2_int(apex1_idx2), y2_int(apex1_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
% text(x1_int(apex1_idx), y1_int(apex1_idx), '  Apex 1', 'FontSize', 10)

% scatter(x1_int(apex2_idx), y1_int(apex2_idx), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(x2_int(apex2_idx2), y2_int(apex2_idx2), 100, colGoldenRod,'filled','MarkerFaceAlpha',.8,'MarkerEdgeAlpha',0, 'marker', 'p')
% text(x1_int(apex2_idx), y1_int(apex2_idx), '  Apex 2', 'FontSize', 10)

% geom apex
% plot(   [x_traj_int(geom_apex1) - sin(theta_traj_int(geom_apex1)) * semi_road_width,   x_traj_int(geom_apex1) + sin(theta_traj_int(geom_apex1)) * semi_road_width],...
%         [y_traj_int(geom_apex1) + cos(theta_traj_int(geom_apex1)) * semi_road_width,   y_traj_int(geom_apex1) - cos(theta_traj_int(geom_apex1)) * semi_road_width], 'color', colSlateGray, 'linewidth', 4)
plot(   [x_traj_int(geom_apex2) - sin(theta_traj_int(geom_apex2)) * semi_road_width,   x_traj_int(geom_apex2) + sin(theta_traj_int(geom_apex2)) * semi_road_width],...
    [y_traj_int(geom_apex2) + cos(theta_traj_int(geom_apex2)) * semi_road_width,   y_traj_int(geom_apex2) - cos(theta_traj_int(geom_apex2)) * semi_road_width], 'color', colSlateGray, 'linewidth', 4)

% turn-in point
% scatter(x1_int(turn_in_points1(1:2:end)), y1_int(turn_in_points1(1:2:end)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
scatter(x2_int(turn_in_points2(1:2:end)), y2_int(turn_in_points2(1:2:end)), 40, colDarkBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
% text(x1_int(turn_in_points1(1:2:end)), y1_int(turn_in_points1(1:2:end)), '  Turn-in point', 'FontSize', 10)
% braking point
% scatter(x1_int(braking_points1(1:2:end)), y1_int(braking_points1(1:2:end)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
scatter(x2_int(braking_points2(1:2:end)), y2_int(braking_points2(1:2:end)), 40, colRed,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
% text(x1_int(braking_points1(1:2:end)), y1_int(braking_points1(1:2:end)), '  Braking point', 'FontSize', 10)

scatter(X(1), Y(1), 20)
scatter(X(1), Y(1), 60)
scatter(X(end), Y(end), 20)
scatter(X(end), Y(end), 60)
scatter(X, Y, 20, colSlateGray,'filled','MarkerFaceAlpha',.2,'MarkerEdgeAlpha',0)
% plot(x_traj2, y_traj2, 'k--')
% grid minor
xlabel('x distance (m)')
ylabel('y distance (m)')
axis equal ;
axis tight ;

xlim([-80 20])
box off

return

%%
fig2            = figure();
fig2.Position   = [100 100 800 600];

subplot(3,1,1)
title('Speed and breaking points')
hold on
%xline(d_traj_int(geom_apex1), 'linewidth', 2, 'color', colSlateGray)
%xline(d_traj_int(geom_apex2), 'linewidth', 2, 'color', colSlateGray)
plot(d_traj_int(d_traj_int<d1(end)),v1_F(d_traj_int<d1(end))*3.6, 'color', colOrangeRed)
plot(d_traj_int(d_traj_int<d2(end)),v2_F(d_traj_int<d2(end))*3.6, 'color', colCornFlowerBlue)
scatter(d_traj_int(braking_points1(1:2:end)), v1_F(braking_points1(1:2:end))* 3.6, ...
    60, colDarkOrange,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
scatter(d_traj_int(braking_points2(1:2:end)), v2_F(braking_points2(1:2:end))* 3.6, ...
    60, colDodgerBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'd')
ax1 = gca;
ax1.XTick = [];
ylabel('Speed (km/h)')
box on
grid minor
subplot(3,1,2)
title('Lateral displacement and apexes')
hold on
xline(d_traj_int(geom_apex1), 'linewidth', 2, 'color', colSlateGray)
xline(d_traj_int(geom_apex2), 'linewidth', 2, 'color', colSlateGray)
yline(4,    'linewidth', 2, 'color', colSlateGray)
yline(-4,   'linewidth', 2, 'color', colSlateGray)
yline(0,   'linewidth', 1, 'color', colSlateGray)
plot(d_traj_int(d_traj_int<d1(end)),n1_int(d_traj_int<d1(end)), 'color', colOrangeRed)
plot(d_traj_int(d_traj_int<d2(end)),n2_int(d_traj_int<d2(end)), 'color', colCornFlowerBlue)
scatter(d_traj_int(apex1_idx), n1_int(apex1_idx), ...
    100, colDarkOrange,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(d_traj_int(apex1_idx2), n2_int(apex1_idx2), ...
    100, colDodgerBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(d_traj_int(apex2_idx), n1_int(apex2_idx), ...
    100, colDarkOrange,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'p')
scatter(d_traj_int(apex2_idx2), n2_int(apex2_idx2), ...
    100, colDodgerBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0, 'marker', 'p')
ylim([-4,4])
ax2 = gca;
ax2.XTick = [];
box on
grid minor
ylabel('Lat dist (m)')
subplot(3,1,3)
title('Relative heading and turn-in points')
hold on
xline(d_traj_int(geom_apex1),'linewidth', 2, 'color', colSlateGray)
xline(d_traj_int(geom_apex2),'linewidth', 2, 'color', colSlateGray)
plot(d_traj_int(d_traj_int<d1(end)),alpha1_F(d_traj_int<d1(end))*180/pi - 90, 'color', colOrangeRed)
plot(d_traj_int(d_traj_int<d2(end)),alpha2_F(d_traj_int<d2(end))*180/pi - 90, 'color', colCornFlowerBlue)
scatter(d_traj_int(turn_in_points1(1:2:end)), alpha1_F(turn_in_points1(1:2:end))*180/pi - 90, ...
    60, colDarkOrange,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
scatter(d_traj_int(turn_in_points2(1:2:end)), alpha2_F(turn_in_points2(1:2:end))*180/pi - 90, ...
    60, colDodgerBlue,'filled','MarkerFaceAlpha',.4,'MarkerEdgeAlpha',0)
xlabel('Midline course curvilinear abscissa (m)')
ylabel('Heading (deg)')
box on
grid minor

return

projection          = projcrs(3857);
[x0_3857,y0_3857]   = projfwd(projection,lat0,long0);

for i=1:length(lat)
    [x_3857(i),y_3857(i)] = projfwd(projection,lat(i),long(i));
end

x_3857 = x_3857 - x0_3857;
y_3857 = y_3857 - y0_3857;


PRO=ell2tm([long, lat],[],long0,lat0,[],[]);
PRO_0=ell2tm([long0, lat0],[],long0,lat0,[],[]);
my_coord = PRO-PRO_0;

x = long - long0;
y = rad2deg(asinh(tan(deg2rad(lat)))) - rad2deg(asinh(tan(deg2rad(lat1))));


[x320,y320,~]   = wgs2utm(lat0,long0,32,'N');

for i=1:length(lat)
    [x32(i),y32(i),~] = wgs2utm(lat(i),long(i),32,'N');
end
x32 = x32 - x320;
y32 = y32 - y320;

% %% GPS DATA
% % mat files have a structure with time|ibi|sbp inside
% files = uigetfile('GPS_traj/.mat', 'Select data files', 'MultiSelect', 'on');
%
% if ~iscell(files)
%     files = {files};
% end %now filename is a cell array regardless of the number of selected files.
%
% XX = [];
% YY = [];
% vv = [];
% aa = [];
% nn = [];
% tt = [];
%
% for i = 1:length(files)
%     % load the files (multiple selection)
%     load(files{i});
%     if i > 5
%         first_guess = first_guess_6_10;
%     else
%         first_guess = first_guess_1_5;
%     end
%     traj_struct = traj_prop([XGPS YGPS], x_traj, y_traj, d_traj, geom_apex1, geom_apex2, first_guess, 'GPS');
%
%     plot(traj_struct.x2_int, traj_struct.y2_int, 'Color', [colSlateGray 0.2], ...
%         'linewidth', 2)
% %     surface_line = surface( 'XData', [traj_struct.x2_int' traj_struct.x2_int'], ... % N.B.  XYZC Data must have at least 2 cols
% %                             'YData', [traj_struct.y2_int' traj_struct.y2_int'], ...
% %                             'ZData', zeros(numel(traj_struct.y2_int),2), ...
% %                             'CData', [traj_struct.v2_F traj_struct.v2_F], ...
% %                             'FaceColor', 'none', ...
% %                             'EdgeColor', 'interp', ...
% %                             'Marker', 'none', ...
% %                             'linewidth', 0.5);
%     % hold on
%
%     XX = [XX traj_struct.x2_int'];
%     YY = [YY traj_struct.y2_int'];
%     aa = [aa traj_struct.alpha2_F'];
%     vv = [vv traj_struct.v2_F];
%     nn = [nn traj_struct.n2_int'];
%     tt = [tt traj_struct.theta2_int'];
%
% end

