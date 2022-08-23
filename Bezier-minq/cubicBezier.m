%
% Function which evaluate bases first derivative of cubic
%
%  P0 * b0(t)' + T0 * bt0(t)' + P1 * b1(t)' + T1 * bt1(t)' 
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
function varargout = cubicBezier( what, varargin )
  switch ( what )
  case 'base'
    [varargout{1:4}] = base(varargin{1}) ;
  case 'base_1'
    [varargout{1:4}] = base_1(varargin{1}) ;
  case 'base_2'
    [varargout{1:4}] = base_2(varargin{1}) ;
  case 'eval'
    varargout{:} = doEval(varargin{:}) ;
  case 'eval_1'
    varargout{:} = doEval_1(varargin{:}) ;
  case 'eval_2'
    varargout{:} = doEval_2(varargin{:}) ;
  case 'error'
    varargout{:} = doEvalError(varargin{:}) ;
  case 'polygon'
    varargout{1} = varargin{1} ;
    varargout{2} = varargin{1}+varargin{2} ;
    varargout{3} = varargin{3}-varargin{4} ;
    varargout{4} = varargin{3} ;
  case {'leastsquares','lsq'}
    [varargout{1:4}] = LSQfixed(varargin{:}) ;
  case 'pp'
    [varargout{1:2}] = makepp(varargin{:}) ;
  otherwise
    varargout{:} = [] ;
  end
end
%
%
%
function [b0,c0,b1,c1] = base( t )
  t1 = 2 .* t - 3 ;
  t2 = t .^ 2;
  b0 = 1 + t1 .* t2 ;
  b1 = 1 - b0 ;
  c0 = 3 .* t .* (1 + t .* (t - 2)) ;
  c1 = (3 .* t - 3) .* t2 ;
end
function [b0,c0,b1,c1] = base_1( t )
  t1 = 9 .* t;
  b0 = (6 .* t - 6) .* t ;
  b1 = (6 - 6 .* t) .* t ;
  c0 = 3 + (t1 -12) .* t ;
  c1 = (t1 - 6) .* t ;
end
function [b0,c0,b1,c1] = base_2( t )
  b0 = 12 .* t - 6 ;
  b1 = -b0 ;
  c0 = 18 .* t - 12 ;
  c1 = c0 + 6 ;
end
%
%
%
function PNTS = doEval( t, P0, T0, P1, T1 )
  [b0,c0,b1,c1] = base( t ) ;
  PNTS = P0 * b0 + T0 * c0 + P1 * b1 + T1 * c1 ;  
end
function PNTS = doEval_1( t, P0, T0, P1, T1 )
  [b0,c0,b1,c1] = base_1( t ) ;
  PNTS = P0 * b0 + T0 * c0 + P1 * b1 + T1 * c1 ;  
end
function PNTS = doEval_2( t, P0, T0, P1, T1 )
  [b0,c0,b1,c1] = base_2( t ) ;
  PNTS = P0 * b0 + T0 * c0 + P1 * b1 + T1 * c1 ;  
end
%
%
%
function sqDist = doEvalError( t, P0, T0, P1, T1, PNTS )
  npts = size(PNTS,1) ;
  uno  = ones( 1, npts ) ;
  % calcolo errore sui singoli nodi
  PNTS1  = doEval(t,P0,T0,P1,T1) ;
  ERR    = sqrt( uno * (PNTS1- PNTS).^2 ) ;
  sqDist = sum(ERR)/npts ; %% calcolo errore
end
%
% Function which given points and tangent build hermite interpolating polinomial
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
function pp = toPoly( L, P0, T0, P1, T1 )
  pp = [ (2*(P0-P1)+3*(T0+T1))/L^3, 3*(P1-P0-2*T0-T1)/L^2, 3*T0/L, P0 ] ;
end
%
% Function which evaluate bases of cubic
%
%  P0 * b0(t) + T0 * bt0(t) + P1 * b1(t) + T1 * bt1(t) 
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
function [ppx,ppy] = makepp( breaks, P0, T0, P1, T1 )
  nseg   = length(breaks)-1 ;
  coeffs = zeros(nseg,2,4) ;
  for k=1:nseg
    coeffs(k,:,:) = toPoly( breaks(k+1)-breaks(k), P0(:,k), T0(:,k), P1(:,k), T1(:,k) ) ;
  end
  ppx = mkpp( breaks, coeffs(:,1,:) ) ;
  ppy = mkpp( breaks, coeffs(:,2,:) ) ;
end


%
% Function which evaluate coefficients P0, P1, T0, T1 for the cubic
%
% S(t) = P0 * b0(t) + T0 * c0(t) + P1 * b1(t) + T1 * c1(t) 
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
% Which minimize the quadratic error
%
% err = sum( (S(tk) - Pk) ^ 2 )
%
% err = sum( (T0 * c0(tk) + T1 * c1(tk) - Qk) ^ 2 )
% Qk = Pk - P0 * b0(tk) - P1 * b1(tk)
%
function [P0, T0, P1, T1] = LSQfixed( t, PNTS, lambda )

  [b0,c0,b1,c1]         = base( t ) ;
  [b0_2,c0_2,b1_2,c1_2] = base_2( t ) ;

  P0 = PNTS(:,1) ;
  P1 = PNTS(:,end) ;

  A = [c0 ; c1] ;
  Q = PNTS - P0*b0 - P1*b1 ;

  A2 = [c0_2 ; c1_2] ;
  Q2 = - P0*b0_2 - P1*b1_2 ;
  
  %RES = ( A * A.' ) \ ( A * Q.' );
  RES = [ A lambda*A2].' \ [Q lambda*Q2].' ; % risolvo ai minimi quadrati
  %RES = A.' \ Q.' ; % risolvo ai minimi quadrati

  T0 = RES(1,:).' ;
  T1 = RES(2,:).' ;
end
%
% Function which evaluate coefficients P0, P1, T0, T1 for the cubic
%
% S(t) = P0 * b0(t) + T0 * c0(t) + P1 * b1(t) + T1 * c1(t) 
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
% Which minimize the quadratic error
%
% err = sum( (S(tk) - Pk) ^ 2 ) 
%
%
function [P0, T0, P1, T1] = LSQfree( t, PNTS )
  MAT  = zeros(4,length(t)) ;
  [b0,c0,b1,c1] = baseCubic( t ) ;
  MAT(1,:) = b0  ;
  MAT(2,:) = c0 ;
  MAT(3,:) = b1  ;
  MAT(4,:) = c1 ;
  A = MAT * MAT.' ;
  b = MAT * PNTS.' ;
  res = A \ b ;
  P0 = res(1,:).' ;  
  T0 = res(2,:).' ;
  P1 = res(3,:).' ;
  T1 = res(4,:).' ;
end
