function y = make_robust(x, varargin)

if length(varargin) < 2
    step_on = 0.1;
    else step_on = varargin(2); 
end

for i = 2:length(x)
    
    if x(i)<=x(i-1)
        x(i)=x(i-1) + step_on;
    end
    
end

y = x;