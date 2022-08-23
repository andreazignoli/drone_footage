%% preamble
clc
close all
clear all

addpath('../GPS/');

%% load testing course

% load the file you exported from GPS visualiser
[FileName,PathName] = uigetfile('../GPS/*.txt','Select the data file with altitude (GPS visualizer)');
disp('http://www.gpsvisualizer.com/convert?output_elevation');

% read the file
fileID = fopen([PathName, FileName]);

nline = 0;
tline = fgetl(fileID);
while ischar(tline)
    tline = fgetl(fileID);
    nline = nline+1;
end

% extraction
kmlwithaltitude = importfilewithaltitude([PathName, FileName]);

lat     = kmlwithaltitude.latitude;
long    = kmlwithaltitude.longitude;
alt     = kmlwithaltitude.altitudem;

[X,Y,Z] = geodetic2enu(lat,long,alt,46.141771665,11.316138614,0,wgs84Ellipsoid);

XGPS = X;
YGPS = Y;

%% fit trajectory
[d_traj, theta_traj, x_traj, y_traj] = fit_xy(X, Y, 2);

%% plot trajectory

hold on
plot(X, Y, 'o')
plot(x_traj, y_traj)
axis equal
