function L = find_transformation(x, v, u)
%% This is made to find the transformation from u to v and vice versa

% u and v are 3-dimensional vectors

% optimise the following parameters
a = x(1);
b = x(2);
c = x(3);
d = x(4);
e = x(5);

% rotation matrices
RZ = rotz(a);

% scale
u1      = [u(:,1) * b u(:,2) * c];
% pure translations
u2      = u1;
u2(:,1) = u1(:,1) + d;
u2(:,2) = u1(:,2) + e;

% rotations
u3  = [u2 zeros(length(u2),1)] * RZ;

% objective function
distance(1) = 0;
for i = 2: length(u)
    distance(i) = distance_points([u3(i); u3(i); 0],[v(i-1); v(i-1); 0]);
end

L = sum(distance.^2);