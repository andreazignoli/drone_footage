function [s, n, theta, x1, y1] = compute_n(Xp,Yp,Xt,Yt,st)
% compute the lateral displacement

xpx         = Xp;
ypx         = Yp;
x_traj      = Xt;
y_traj      = Yt;
d_traj      = st;
theta       = zeros(size(xpx));
theta(1)    = theta(2);

for j=1:length(xpx)
    
    d0  = inf;
    
    % find the closest point
    for i=2:(length(d_traj)-1)
        
        d = distance_points([xpx(j),ypx(j),0],[x_traj(i),y_traj(i),0]);
        
        if d < d0
            d0          = d;
            min_idx(j)  = i;
        end
        
    end
    
    v = cross([xpx(j) - x_traj(min_idx(j)), ...
        ypx(j) - y_traj(min_idx(j)),0], [x_traj(min_idx(j)+1) - x_traj(min_idx(j)), ...
        y_traj(min_idx(j)+1) - y_traj(min_idx(j)), 0]);
    s_COM(j) = d_traj(min_idx(j));
    n_COM(j) = d0 * sign(v(3));
    
    if j > 1
        
        % make angles always positive
        
        absolute_heading_trajectory = atan2(ypx(j)-ypx(j-1), xpx(j)-xpx(j-1));
        if absolute_heading_trajectory< pi
            absolute_heading_trajectory = absolute_heading_trajectory + 2 * pi;
        end
        %
        absolute_midline_heading = atan2(y_traj(min_idx(j)) - y_traj(min_idx(j)-1), ...
            x_traj(min_idx(j)) - x_traj(min_idx(j)-1));
        if absolute_midline_heading<0
            absolute_midline_heading = absolute_midline_heading + 2 * pi;
        end
        %
        theta(j) = absolute_heading_trajectory; % - absolute_midline_heading;
    end
    
end

s       = s_COM(min_idx>2 & min_idx<(length(d_traj)-1));
n       = n_COM(min_idx>2 & min_idx<(length(d_traj)-1));
x1      = xpx(min_idx>2 & min_idx<(length(d_traj)-1));
y1      = ypx(min_idx>2 & min_idx<(length(d_traj)-1));
theta   = theta(min_idx>2 & min_idx<(length(d_traj)-1));

s(abs(diff(n))>0.5)     = [];
theta(abs(diff(n))>0.5) = [];
x1(abs(diff(n))>0.5)    = [];
y1(abs(diff(n))>0.5)    = [];
n(abs(diff(n))>0.5)     = [];

s(abs(diff(theta))>0.5)     = [];
n(abs(diff(theta))>0.5)     = [];
x1(abs(diff(theta))>0.5)    = [];
y1(abs(diff(theta))>0.5)    = [];
theta(abs(diff(theta))>0.5) = [];

n(diff([0 s])<=0)       = [];
theta(diff([0 s])<=0)   = [];
x1(diff([0 s])<=0)      = [];
y1(diff([0 s])<=0)      = [];
s(diff([0 s])<=0)       = [];

EOF=1;
