function [x,u] = optimal_filter( t, d, lambda )
if size(t,1) < size(t,2)
    t = t';
end
if size(d,1) < size(d,2)
    d = d';
end
h  = 0.5*[ t(2)-t(1) ; t(3:end)-t(1:end-2) ; t(end)-t(end-1)] ;
dg = lambda./h ;
% symmetric tri-diagonal system
a  = -dg(2:end) ;
b  = diff(t)+dg(1:end-1)+dg(2:end) ;
u  = diff(d) ;
% solution of the sistem
n  = length(u);
for j = 1:n-1
    mu = a(j)/b(j);
    b(j+1) = b(j+1) - mu*a(j);
    u(j+1) = u(j+1) - mu*u(j);
end
u(n) = u(n)/b(n);
for j = n-1:-1:1
    u(j) = (u(j)-a(j)*u(j+1))/b(j);
end
% retrieving solution
x    = zeros(size(d)) ;
x(1) = d(1)+lambda*u(1)/h(1) ;
for i=1:n
    x(i+1)=x(i)+(t(i+1)-t(i))*u(i) ;
end
end
