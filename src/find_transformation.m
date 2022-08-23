function L = find_transformation(x, v, u)
%% This is made to find the transformation from u to v and vice versa

% u and v are 3-dimensional vectors

% optimise the following parameters
a = x(1);
b = x(2);
c = x(3);
d = x(4);
e = x(5);
f = x(6);
g = x(7);
h = x(8);
i = x(9);

% rotation matrices
RX = rotx(a);
RY = roty(b);
RZ = rotz(c);

% scale
u1      = [u(:,1) * d u(:,2) * e u(:,3) * f];
% pure translations
u2      = u1;
u2(:,1) = u1(:,1) + g;
u2(:,2) = u1(:,2) + h;
u2(:,3) = u1(:,3) + i;

% rotations
u3  = u2 * RX * RY * RZ;

% objective function
distance(1) = 0;
for i = 2: length(u)
    distance(i) = distance_points([u3(i); u3(i); 0],[v(i-1); v(i-1); 0]);
end

L = sumsqr(distance);