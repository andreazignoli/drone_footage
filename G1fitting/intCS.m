%
%  Calcolo 
%
%  int( t^k * cos( (a/2)*t^2 + b*t + c, t=0..1)
%  int( t^k * cos( (a/2)*t^2 + b*t + c, t=0..1)
%
function [intC,intS] = intCS( nk, a, b, c, t )
  npt  = length(t) ;
  intC = zeros(nk,npt) ;
  intS = zeros(nk,npt) ;
  for k=1:length(t)
    tt    = [t(k),t(k)^2,t(k)^3] ;
    [C,S] = intXY( nk, a*tt(2), b*tt(1), c ) ;
    for j=1:nk
      intC(j,k) = tt(j)*C(j) ;
      intS(j,k) = tt(j)*S(j) ;
    end
  end
end
