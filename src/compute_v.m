function [d_tot, v] = compute_v(X,Y,Tc)

v    = 0;
d(1) = 0;

if length(Tc) == 1
    Tc = ones(length(X), 1) * Tc;
end

for i = 2:length(Y)
    
    d(i)    = distance_points([X(i),Y(i),0],[X(i-1),Y(i-1),0]);
    v(i)    = d(i)/Tc(i);
    
end

v(d==0) = [];
d(d==0) = [];

v(1)  = v(2);
d_tot = cumsum(d);

