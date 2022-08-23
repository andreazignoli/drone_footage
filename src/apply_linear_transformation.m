function X = apply_linear_transformation(Y, x_opt)

% extract values
a = x_opt(1);
b = x_opt(2);
c = x_opt(3);

d = x_opt(4);
e = x_opt(5);
f = x_opt(6);

g = x_opt(7);
h = x_opt(8);
i = x_opt(9);

% rotation matrices
RX = rotx(a);
RY = roty(b);
RZ = rotz(c);

% scale
u1      = [Y(:,1) * d Y(:,2) * e Y(:,3) * f];

% pure translations
u2      = u1;
u2(:,1) = u1(:,1) + g;
u2(:,2) = u1(:,2) + h;
u2(:,3) = u1(:,3) + i;

% rotations
X  = u2 * RX * RY * RZ;


