%
%  Calcolo 
%
%  int( t^k * cos( (a/2)*t^2 + b*t + c, t=0..1)
%  int( t^k * cos( (a/2)*t^2 + b*t + c, t=0..1)
%  7.32609e-16 -3.14159
function [X,Y] = intXY( nk, a, b, c )

  absa = abs(a) ;
  epsi = 0.0001 ;

  X = zeros( nk, 1 ) ;
  Y = zeros( nk, 1 ) ;

  if absa < epsi % caso a piccolo
    [X,Y] = evalXYaSmall( nk, a, b, 4 ) ;
  else
    [X,Y] = evalXYaBig( nk, a, b ) ;
  end

  cc = cos(c) ;
  ss = sin(c) ;

  for k=1:nk
    xx = X(k) ;
    yy = Y(k) ;
    X(k) = xx*cc-yy*ss ;
    Y(k) = xx*ss+yy*cc ;
  end

end
%
%
%
function [X,Y] = evalXYaBig( nk, a, b )
  s       = sign(a) ;
  z       = sqrt(abs(a)/pi) ;
  ell     = s*b/sqrt(abs(a)*pi) ;
  g       = -s*b^2/(2*abs(a)) ;
  cg      = cos(g) ;
  sg      = sin(g) ;
  [Cl,Sl] = FresnelCS(ell) ;
  [Cz,Sz] = FresnelCS(ell+z) ;
  dC0     = Cz - Cl ;
  dS0     = Sz - Sl ;
  X       = zeros(nk,1) ;
  Y       = zeros(nk,1) ;
  X(1)    = (cg * dC0 - s * sg * dS0)/z ;
  Y(1)    = (sg * dC0 + s * cg * dS0)/z ;
  if nk > 1
    t    = a/2+b ;
    ct   = cos(t) ;
    st   = sin(t) ;
    X(2) = (   st   - b*X(1) ) / a ;
    Y(2) = ( (1-ct) - b*Y(1) ) / a ;
    if nk > 2
      for k=1:nk-2
        X(k+2) = (st-b*X(k+1)-k*Y(k))/a ;
        Y(k+2) = (k*X(k)-b*Y(k+1)-ct)/a ;
      end
    end
  end
end
%
%
%
function res = S(mu,nu,b)
  tmp = 1/((mu+nu+1)*(mu-nu+1)) ;
  res = tmp ;
  for n=1:100
    tmp = tmp * (-b/(2*n+mu-nu+1)) * (b/(2*n+mu+nu+1)) ;
    res = res + tmp ;
    if abs(tmp) < abs(res) * 1e-50
      break ;
    end;
  end
end
%
%
%
function [X,Y] = evalXYazero( nk, b )
  X  = zeros(nk,1) ;
  Y  = zeros(nk,1) ;
  sb = sin(b) ;
  cb = cos(b) ;
  if abs(b) < 1e-5
    X(1) = 1-b^2/6*(1-b^2/20) ;
    Y(1) = (b/2)*(1-b^2/6*(1-b^2/30)) ;
  else
    X(1) = sb/b ;
    Y(1) = (1-cb)/b ;
  end
  A = b*sb ;
  D = sb-b*cb ;
  B = b*D ;
  C = -b^2*sb ;
  for k=1:nk-1
    X(k+1) = ( k*A*S(k+1/2,3/2,b) + B*S(k+3/2,1/2,b) + cb )/(1+k) ;
    Y(k+1) = ( C*S(k+3/2,3/2,b) + sb ) / (2+k) + D*S(k+1/2,1/2,b) ;
  end
end

%
% VERSIONE STABILE
%
function [X,Y] = evalXYaSmall( nk, a, b, p )
  [X0,Y0] = evalXYazero( nk + 4*p + 2, b ) ;

  for j=1:nk
    X(j) = X0(j)-a*Y0(j+2)/2 ;
    Y(j) = Y0(j)+a*X0(j+2)/2 ;
  end

  t  = 1 ;
  aa = -a^2 ;
  for n=1:p
    t = t*aa/(16*n*(2*n-1)) ;
    for j=1:nk
      X(j) = X(j) + t*(X0(4*n+j)-a*Y0(4*n+j+2)/(2*(2*n+1))) ;
      Y(j) = Y(j) + t*(Y0(4*n+j)+a*X0(4*n+j+2)/(2*(2*n+1))) ;
    end
  end
end
