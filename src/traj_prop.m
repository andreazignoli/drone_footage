function traj_struct = traj_prop(trajectory, x_traj, y_traj, d_traj, geom_apex1, geom_apex2, first_guess, drone_or_GPS)

t1      = [trajectory(:,1) trajectory(:,2) zeros(length(trajectory(:,1)), 1)];

if strcmp(drone_or_GPS, 'drone')
    t11             = apply_linear_transformation(t1, first_guess);
    bezier_factor   = 1;
    Tc              = 1/29.9700 * 2; % sampling time for computing speed
end

if strcmp(drone_or_GPS, 'GPS')
    t11 = flip(t1);
    bezier_factor = 2;
    Tc = 1; % sampling time for computing speed
end

if strcmp(drone_or_GPS, 'PINS')
    t11             = flip(t1);
    bezier_factor   = 1;
    Tc              = 1; % sampling time for computing speed
end

x       = t11(:,1);
y       = t11(:,2);

traj_struct = struct();

% traj int
% x_traj_int      = interp1(d_traj,x_traj,d_traj:0.5:d_traj(end));
% y_traj_int      = interp1(d_traj,y_traj,d_traj:0.5:d_traj(end));
d_traj_int      = interp1(d_traj,d_traj,d_traj:0.5:d_traj(end));
% theta_traj_int  = interp1(d_traj,theta_traj,d_traj:0.5:d_traj(end), 'PCHIP');

[traj_struct.d_traj2, traj_struct.theta_traj2, traj_struct.x_traj2, traj_struct.y_traj2] = fit_xy(x,y,bezier_factor);
% compute velocity and displacement

[traj_struct.d2, traj_struct.v2] = compute_v(x,y,Tc);
[traj_struct.s2, traj_struct.n2, traj_struct.theta2, traj_struct.x2, traj_struct.y2] = compute_n(traj_struct.x_traj2,traj_struct.y_traj2,x_traj,y_traj,d_traj);

traj_struct.s2_int      = interp1(traj_struct.s2,traj_struct.s2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');
traj_struct.theta2_int  = interp1(traj_struct.s2,traj_struct.theta2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');
traj_struct.x2_int      = interp1(traj_struct.s2,traj_struct.x2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');
traj_struct.y2_int      = interp1(traj_struct.s2,traj_struct.y2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');
traj_struct.n2_int      = interp1(traj_struct.s2,traj_struct.n2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');
traj_struct.alpha2_int  = interp1(traj_struct.s2,traj_struct.theta2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');

traj_struct.alpha2_F    = optimal_filter(d_traj_int(d_traj_int<traj_struct.d2(end)), traj_struct.alpha2_int, 20);
traj_struct.v2_int      = interp1(traj_struct.d2,traj_struct.v2,d_traj_int(d_traj_int<traj_struct.d2(end)), 'PCHIP');
traj_struct.v2_F        = optimal_filter(d_traj_int(d_traj_int<traj_struct.d2(end)), traj_struct.v2_int, 20);

[traj_struct.apex1_val2, traj_struct.apex1_idx2]    = min(traj_struct.n2_int(d_traj_int<100));
[traj_struct.apex2_val2, traj_struct.apex2_idx2]    = max(traj_struct.n2_int(d_traj_int>100 & d_traj_int<traj_struct.d2(end)));
traj_struct.apex2_idx2 = traj_struct.apex2_idx2 + length(d_traj_int( 1:find( d_traj_int> 100, 1 ) )) -1;

traj_struct.turn_in_points2 = findchangepts(traj_struct.alpha2_F,   'Statistic','linear','MaxNumChanges', 4);
traj_struct.braking_points2 = findchangepts(traj_struct.v2_F,       'Statistic','linear','MaxNumChanges', 6);

traj_struct.turn_in_distance12   = geom_apex1    - d_traj_int(traj_struct.turn_in_points2(1));
traj_struct.braking_distance12   = geom_apex1    - d_traj_int(traj_struct.braking_points2(1));
traj_struct.apex_distance12      = d_traj_int(traj_struct.apex1_idx2)    - geom_apex1 ;

traj_struct.turn_in_distance22   = geom_apex2    - d_traj_int(traj_struct.turn_in_points2(3));
traj_struct.braking_distance22   = geom_apex2    - d_traj_int(traj_struct.braking_points2(4));
traj_struct.apex_distance22      = d_traj_int(traj_struct.apex2_idx2)    - geom_apex2 ;

