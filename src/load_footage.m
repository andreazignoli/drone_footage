clc
clear all
close all

addpath('../Bezier-minq');
addpath('../G1fitting');
addpath('../functions');
addpath('../movie');
addpath('../GPS');
addpath(genpath('/Users/andreazignoli/matlab-toolbox/'))

Figure_Settings
% just for this video
load M_traj.mat
load midline_XY.mat

box_dimension_x = 30;
box_dimension_y = 30;

%% load file
[file, path]            = uigetfile('../movie/*.mp4'); % e.g. 'drone_footage_1.mp4'
v                       = VideoReader([path, file]);

%% loop frames
fig                 = figure();
currAxes            = axes;
frame               = readFrame(v);
image(frame, 'Parent', currAxes);
currAxes.Visible    = 'off';

% first you need to set the Point 0 (lat0 lon0 in the WGS84)
% second you need to set the Point 1 (NORTH)
% [x0,y0] = ginput(41);

% second you need to select the first frame with visible CoM
[x,y]   = ginput(1);
[x1,y1] = ginput(1);
xi = x;
yi = y;
xf = x1;
yf = y1;
delta_x = x1-x;
delta_y = y1-y;

% close the window and start iteration
close all

%% make video
% create the video writer with 1 fps
writerObj               = VideoWriter('myVideo.mp4', 'MPEG-4');
writerObj.FrameRate     = 30;
% open the video writer
open(writerObj);

%% load file again

v           = VideoReader([path, file]);
trajectory  = [];

%% flow the video and follow the cyclist
% fig         = figure('position', [100 100 1200 600]);
fig         = figure();
currAxes    = axes;
max_area    = 220;
i           = 0;
i0          = 2;
delta_x(1:i0) = x1-x;
delta_y(1:i0) = y1-y;

while hasFrame(v)
    
    i                   = i+1
    vidFrame            = readFrame(v);
    J0(:,:,i*3-2:i*3)   = imrotate(vidFrame, 0);
    
    if i > i0
        
        vidFrame    = readFrame(v);
        J           = imrotate(vidFrame, 0);
        
        IM_diff = J - J0(:,:,(i-i0)*3-2:(i-i0)*3);
        
        I       = IM_diff(max(1,round(y)-box_dimension_x):round(y)+box_dimension_x, ...
            round(x)-box_dimension_y:round(x)+box_dimension_y, :);
        
        I_or    = J(max(1,round(y)-box_dimension_x):round(y)+box_dimension_x, ...
            round(x)-box_dimension_y:round(x)+box_dimension_y, :);
        
        sensitivity                             = 0.85;
        my_image                                = wiener2(imadjust(rgb2gray(I_or)),[14 14]);
        my_image(my_image<200)                  = 0;
        [centersBright, radiiBright, metric]    = imfindcircles(my_image,[4 20],'ObjectPolarity','bright', 'sensitivity', sensitivity);
        
        if ~isempty(centersBright)
            centroid = centersBright(1,:);
        end
        
        while isempty(centersBright)
            sensitivity = sensitivity * 1.1;
            [centersBright, radiiBright, metric] = imfindcircles(my_image,[4 20],'ObjectPolarity','bright', 'sensitivity', min(sensitivity, 1));
            if ~isempty(centersBright)
            centroid = centersBright(1,:);
        end
        end
        
        % Ibw = rgb2gray(I);
        % BW  = imcomplement(im2bw(I));
        % BW      = imbinarize(imadjust(rgb2gray(imgaussfilt(I))));
        % BW      = imshow(imadjust(rgb2gray(I+I_or)));
        % BW      = bwmorph(BW,'thicken', 2);
        % BW      = bwmorph(BW,'bridge');
        
        % BW    = imcomplement(I3);
        
        % e     = edge(BW, 'canny');
        % imshow(e);
        
        %radii = 10:1:40;
        %hh = circle_hough(e, radii, 'same', 'normalise');
        %peaks = circle_houghpeaks(hh, radii, 'nhoodxy', 15, 'nhoodr', 21, 'npeaks', 1);
        
        %% Look at the results
        % We draw the circles found on the image, using both the positions and the
        % radii stored in the |peaks| array. The |circlepoints| function is
        % convenient for this - it is also used by |circle_hough| so comes with it.
        
        %         imshow(BW);
        %         hold on;
        %         for peak = peaks
        %             [xC, yC] = circlepoints(peak(3));
        %             plot(xC+peak(1), yC+peak(2), 'g-');
        %         end
        %         hold off
        % centroid = peaks(1:2);
        
        %         % find centroid
        %         % mask = bwareafilt(BW, [200, inf]);
        %         st = regionprops(BW,...
        %             'centroid', 'Area');
        %
        %         % find centroid with higher pixel values
        %         centroid_approx = round(cat(1,st.Centroid));
        %         area_idx        = 0;
        %         centroid_idx    = 1;
        %
        %         for q =1:length(centroid_approx(:,1))
        %             [row, colmns]   = size(BW);
        %             mean_value(q)   = mean(BW(max(centroid_approx(q,1)-10,1):min(centroid_approx(q,1)+10, row), max(centroid_approx(q,2)-10,1):min(centroid_approx(q,2)+10, colmns)), 'all');
        %             if mean_value(q) >= area_idx
        %                 area_idx        = mean_value;
        %                 centroid_idx    = q;
        %             end
        %         end
        %
        %         % [centroid, radiiBright] = imfindcircles(BW,[6 60],'ObjectPolarity','bright');
        %
        %         shapes              = [cat(1,st.Area) cat(1,st.Centroid)];
        %         [max_area, peak]    = max(shapes(:,1));
        %         centroid = centroid_approx(centroid_idx, :);
        
        if max_area > 800 || i == 360
            break
        end
        
        % image(vidFrame, 'Parent', currAxes);
        
        if i == (i0+1)
            h     = imshow(J);
        else
            set(h, 'CData', J);
        end
        
        currAxes.Visible = 'off';
        hold on
        pause(1/v.FrameRate);
        
        % trajectory update
        xi = x;
        yi = y;
        
        x = x + centroid(1) - box_dimension_x;
        y = y + centroid(2) - box_dimension_y;
        
        xf = x;
        yf = y;
        
        % variation of position
        delta_x(i+1-i0) = x1-x;
        delta_y(i+1-i0) = y1-y;
        
        % check if something went wrong
        if abs(delta_x(i+1-i0)) > 25 | abs(delta_y(i+1-i0) > 25)
            here=0;
        end
        
        trajectory  = [trajectory; [x, y]];
        
        % plot update
        % scatter(x, y, 600,  colRed, 'o')
        % scatter(x, y, 60,   colOrangeRed, 'o')
        
        % lj          = plot(M_traj(:,1), M_traj(:,2), '--', 'color', colDarkYellow, 'linewidth', 2);
        % lj.Color    = [colDarkYellow 0.6];
        % text(M_traj(1,1), M_traj(1,2) + 20, 'Midline', 'color', colDarkYellow);
        lh          = plot(trajectory(:,1), trajectory(:,2), 'color', colDodgerBlue, 'linewidth', 2);
        text(trajectory(1,1), trajectory(1,2) + 20, 'Cycling trajectory', 'color', colDodgerBlue);
        lh.Color    = [colDodgerBlue 0.4];
        
        % scatter(trajectory(:,1), trajectory(:,2), 10)
        
        % write video
        writeVideo(writerObj, getframe(gcf));
        
    end
    
end

% close the writer object
close(writerObj);
close all

return

%% process the trajectory

load trial_1_midline.mat
load midline_XY.mat

tform                   = fitgeotrans([x0 y0], [X Y], 'similarity');

% real_trajectory         = [trajectory ones(length(trajectory),1)] * tform.T;
midline_video           = [x0 y0 ones(length(x0),1)] * tform.T;



%%
% figure()
close all
hold on
plot(X, Y, 'r')
plot(midline_video(:,1), midline_video(:,2), 'k')

axis equal

xAERIAL = real_trajectory(:,1);
yAERIAL = real_trajectory(:,2);

save(['traj_', file(1:end-4)], 'xAERIAL', 'yAERIAL');








