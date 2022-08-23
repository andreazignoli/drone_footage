%% get solution data

clc
close all
clear all

addpath(genpath('/Users/andreazignoli/matlab-toolbox/toolbox'));
Figure_Settings

%% load the data

[txt_file,txt_file_path] = uigetfile('../../redebus/ocp-road-sprint-bio-model/data/.txt', 'MultiSelect', 'on');
D = importdata([txt_file_path, txt_file]);

%% get the vars
zeta        = getData(D,'zeta');
time        = getData(D,'t');
power       = getData(D,'Power');
alpha       = getData(D,'alpha');
vel         = getData(D,'u');
xLeftEdge   = getData(D,'xLeftEdge');
yLeftEdge   = getData(D,'yLeftEdge');
xRightEdge  = getData(D,'xRightEdge');
yRightEdge  = getData(D,'yRightEdge');
xLane       = getData(D,'xLane');
yLane       = getData(D,'yLane');
xCoM        = getData(D,'xCoM');
yCoM        = getData(D,'yCoM');
phi         = getData(D,'phi');
slope       = getData(D,'slope');
n           = getData(D,'n');
steer       = getData(D,'steer');
ax          = getData(D,'ax');
ay          = getData(D,'ay');
brake       = power;

power(power<0)  = 0;
brake(brake>0)  = 0;
brake           = -brake;

race_time = seconds(time(end));
race_time.Format = 'hh:mm:ss'

save traj_PINS xCoM yCoM xLeftEdge yLeftEdge xRightEdge yRightEdge xLane yLane vel zeta alpha

return

%% HR estimation

MM      = unique([time, power],'rows');
G_power = griddedInterpolant(MM(:,1), MM(:,2));

% P(1) = tau2;
% P(2) = s;
% P(3) = VO2MAX;
% P(4) = VO2R;
% P(5) = CP;
% P(6) = VD;
% P(7) = Delta;
% P(8) = tau3;

P = [43; 8.67; 5000; 850; 420; 897; 64; 180];

y0 = [450 450];
time_sim(1) = 0;
Tc = 1;

for i = 1:time(end)
    time_sim(i+1) = time_sim(i) + Tc;
    [t,y] = ode45(@(t,x) HR_model(t,x,G_power(time_sim(i)),P), [time_sim(i) time_sim(i+1)], y0);
    y0 = y(end, :);
    HR(i) = round((y0(1) + y0(2) + P(4)) * 0.038);
    power_(i) = G_power(time_sim(i));
    time_sim(i) = t(end);
end

%% load GPS data

C = importdata('../../redebus/road_midline_redebus_UP.txt');
[X,Y,Z] = geodetic2enu(C.data(:,1),C.data(:,2),C.data(:,3),C.data(1,1),C.data(1,2),0,wgs84Ellipsoid);
dist(1) = 0;
for i = 2:length(X)
   dist(i) = sqrt( (X(i) - X(i-1))^2 + (Y(i) - Y(i-1))^2 + (Z(i) - Z(i-1))^2 ); 
end
s = cumsum(dist);
F = griddedInterpolant(s, Z);
alt = F(zeta);
ZF = optimal_filter(s, Z, 20);

%% trajectory

z_min = zeta(1);
z_max = zeta(end);

%% plot
% load colors
Figure_Settings

figure()
hold on
% plot_shaded(zeta(1:100:end), power(1:100:end), 'alpha', 0.5, 'color', colDodgerBlue)
% plot_shaded(zeta(1:100:end), brake(1:100:end), 'alpha', 0.75, 'color', colOrangeRed)
% plot_shaded(zeta(1:100:end), EAn(1:100:end), 'alpha', 0.75, 'color', colForestGreen)
plot_shaded(s(1:1:end), ZF(1:1:end), 'alpha', 0.75, 'color', colGreen)
set(gca, 'color', 'none');
set(gcf, 'color', 'none');

fig1 = gcf();
fig1.Position = [100 100 1024 400];
fig1.CurrentAxes.YTick = [];
fig1.CurrentAxes.XTick = [];
fig1.CurrentAxes.YAxis.Visible = 'off';
fig1.CurrentAxes.XAxis.Visible = 'off';

export_fig('altitude', '-dpng', '-transparent', '-r300');


%% DATA

figure('Position', [100 0 600 1200])
subplot(8,1,1)
hold on
plot(s, Z, 'k')
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
ylabel('El m')
xlim([0 max(s)])
grid minor
fill_between(s, Z.*0, Z, [], colOrangeRed);
yyaxis right
hold on
plot(zeta, slope)
subplot(8,1,2)
hold on
plot(zeta, vel * 3.6, 'linewidth', 1, 'color', colDodgerBlue)
% plot(results1.dist_cut, results1.speed_cut * 3.6, 'linewidth', 1, 'color', colOrangeRed)
ylabel('v km/h')
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
xlim([0 max(s)])
grid minor
yLimits = get(gca,'YLim');

subplot(8,1,3)
hold on
plot(zeta, brake, 'linewidth', 1, 'color', colRed)
plot(zeta, power, 'linewidth', 1, 'color', colDodgerBlue)
% plot(results1.dist_cut, results1.Watts_cut, 'linewidth', 1, 'color', colOrangeRed)
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
ylabel('P W')
xlim([0 max(s)])
grid minor

subplot(8,1,4)
hold on
plot(zeta, steer * 180/pi, 'color', colDodgerBlue);
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
ylabel('\delta deg')
xlim([0 max(s)])
grid minor

subplot(8,1,5)
hold on
plot(zeta, ax./9.81, 'linewidth', 1, 'color', colDodgerBlue)
ylabel('a_x g')
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
xlim([0 max(s)])
grid minor

subplot(8,1,6)
hold on
plot(zeta, ay./9.81, 'linewidth', 1, 'color', colDodgerBlue)
ylabel('a_y g')
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
xlim([0 max(s)])
grid minor

subplot(8,1,7)
hold on
plot(zeta, phi*180/pi, 'linewidth', 1, 'color', colDodgerBlue)
ylabel('\Phi deg')
set(gca,'xtick',[]);
set(gca,'xcolor',[1 1 1])
ylim([-30 30])
xlim([0 max(s)])
grid minor

subplot(8,1,8)
hold on
plot(zeta, n, 'linewidth', 1, 'color', colDodgerBlue)
% plot(results1.dist_cut, results1.Watts_cut, 'linewidth', 1, 'color', colOrangeRed)
ylabel('n m')
xlabel('   Dist')
xlim([0 max(s)])
grid minor
b=axes('Position',[0.130 .05 .8 1e-12]);
set(b,'Units','normalized');
set(b,'Color','none');
set(b,'xlim',[0 max(time)]);
xlabel(b, 'Time (s)')
set(gca,'XMinorTick','on','YMinorTick','off')
pos = get(gca, 'Position');
%pos(1) = 0.055;
%pos(2) = 0.1;
set(gca, 'Position', pos);


%% MAP

zeta_0 = zeta(1);
zeta_f = zeta(end);

figure('position', [10 10 600 800])
hold on
% left edge
plot(xLeftEdge(zeta>zeta_0 & zeta<zeta_f), yLeftEdge(zeta>zeta_0 & zeta<zeta_f), 'linewidth', 2, 'color', colSilver)
plot(xRightEdge(zeta>zeta_0 & zeta<zeta_f), yRightEdge(zeta>zeta_0 & zeta<zeta_f),'linewidth', 2, 'color', colSilver)
plot(xLane(zeta>zeta_0 & zeta<zeta_f), yLane(zeta>zeta_0 & zeta<zeta_f), '--', 'linewidth', 2, 'color', colSilver)
% contour(X,Y,Z);
% plot(xCoM(zeta>zeta_0 & zeta<zeta_f), yCoM(zeta>zeta_0 & zeta<zeta_f), 'linewidth', 4, 'color', colOrangeRed);
surface('XData', [xCoM(zeta>zeta_0 & zeta<zeta_f) xCoM(zeta>zeta_0 & zeta<zeta_f)],             ... % N.B.  XYZC Data must have at least 2 cols
    'YData', [yCoM(zeta>zeta_0 & zeta<zeta_f) yCoM(zeta>zeta_0 & zeta<zeta_f)],             ...
    'ZData', zeros(numel(yCoM(zeta>zeta_0 & zeta<zeta_f)),2), ...
    'CData', [vel(zeta>zeta_0 & zeta<zeta_f)*3.6 vel(zeta>zeta_0 & zeta<zeta_f)*3.6],             ...
    'FaceColor', 'none',        ...
    'EdgeColor', 'interp',      ...
    'Marker', 'none', ...
    'linewidth', 4);

gree2orange = create_mapNcol([colGreen; colDodgerBlue; colOrangeRed ], 100);
colormap(gree2orange);
colorbar;
set(gca, 'color', 'none');
set(gcf, 'color', 'none');
fig1 = gcf();
fig1.CurrentAxes.YTick = [];
fig1.CurrentAxes.XTick = [];
fig1.CurrentAxes.YAxis.Visible = 'off';
fig1.CurrentAxes.XAxis.Visible = 'off';

% load XYZ_Yates.mat;
% plot(X, Y, 'm');
axis equal

% th=0;
% for i = 1:length(zeta)
%     if zeta(i)>th
%         text(xLane(i), yLane(i), ['  ', num2str(th/1000)])
%         plot(xLane(i), yLane(i), 'or')
%         th=(th+500);
%     end
% end 

axis off
export_fig('trajectory', '-dpng', '-transparent', '-r300');
                                       


