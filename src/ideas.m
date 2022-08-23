clc
clear all

x = -5:0.01:5;

a = 0.8;
b = 1.2;

tol = 0.00001;
eps = 0.00001;

delta = tol * (b - a)/2;
c = -eps / log(cos(pi * (-b + 2 * delta + a) / (-b + a) / 0.2e1));

f = - 10^6 * c * log( cos((pi/2 * (2 * x - (b + a))/(b - a))));

% plot(x,f)
% xlim([a,1])

h   = tol;
H   = 0.8 - h;
A1  = eps/H^2;
A2  = H^2 -eps/(H^2*h^2);
res = 0;

for i = 1:length(x)
    
    if x(i)<= -H
        res(i) = (x(i) +H)^2;
    end
    
    if (x(i)> -H && x(i)< H)
        res(i) = 0;
    end
    
    if x(i)>= H
        res(i) = (x(i) -H)^2;
    end
    
    f2(i) = + A1 * (x(i))^2 - A2 * res(i);
end

close all
figure()
hold on
plot(x, f2)
xlim([-1 1])

