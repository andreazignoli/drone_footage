%
% Function which evaluate functions on quadratic bezier
%
%  P0 * b0(t) + P1 * b1(t) + P2 * b2(t) 
%
function varargout = quadraticBezier( what, varargin )
  switch ( what )
  case 'base'
    varargout{:} = base(varargin{1}) ;
  case 'base_1'
    varargout{:} = base_1(varargin{1}) ;
  case 'base_2'
    varargout{:} = base_2(varargin{1}) ;
  case 'eval'
    varargout{:} = doEval(varargin{:}) ;
  case 'eval_1'
    varargout{:} = doEval_1(varargin{:}) ;
  case 'eval_2'
    varargout{:} = doEval_2(varargin{:}) ;
  case 'error'
    varargout{:} = doEvalError(varargin{:}) ;
  case 'pp'
    [varargout{1:2}] = makepp(varargin{:})
  case {'leastsquares','lsq'}
    [varargout{1:3}] = LSQfixed(varargin{:}) ;
  otherwise
    varargout{:} = [] ;
  end
end
%
%
%
function [b0,b1,b2] = base( t )
  b0 = (1-t).^2 ;
  b1 = 2*t.*(1-t) ;
  b2 = t.^2 ;
end
function [b0,b1,b2] = base_1( t )
  b0 = 2*(t-1) ;
  b1 = 2-4*t ;
  b2 = 2*t ;
end
function [b0,b1,b2] = base_2( t )
  b0  = 2*ones(size(t)) ;
  b1  = -2*b0 ;
  b2  = b0 ;
end
%
%
%
function PNTS = doEval( t, P0, P1, P2 )
  [b0,b1,b2] = base( t ) ;
  PNTS = P0 * b0 + P1 * b1 + P2 * b2 ;  
end
function PNTS = doEval_1( t, P0, P1, P2  )
  [b0,b1,b2] = base_1( t ) ;
  PNTS = P0 * b0 + P1 * b1 + P2 * b2 ;  
end
function PNTS = doEval_2( t, P0, P1, P2  )
  [b0,b1,b2] = base_2( t ) ;
  PNTS = P0 * b0 + P1 * b1 + P2 * b2 ;  
end
%
%
%
function sqDist = doEvalError( t, P0, P1, P2, PNTS )
  npts = size(PNTS,1) ;
  uno  = ones( 1, npts ) ;
  % calcolo errore sui singoli nodi
  PNTS1  = doEval(t,P0,P1,P2) ;
  ERR    = sqrt( uno * (PNTS1- PNTS).^2 ) ;
  sqDist = sum(ERR)/npts ; %% calcolo errore
end
%
% Function which given points and tangent build hermite interpolating polinomial
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
function pp = toPoly( L, P0, P1, P2 )
  pp = [ (P0-2*P1+P2)/L^2, 2*(P1-P0)/L, P0 ] ;
end
%
% Function which evaluate bases of cubic
%
%  P0 * b0(t) + T0 * bt0(t) + P1 * b1(t) + T1 * bt1(t) 
%
% where P0 and P1 are initial and final point of the cubic
% and T0 and T1 are direction for the tangents.
%
function [ppx,ppy] = makepp( breaks, P0, P1, P2 )
  nseg   = length(breaks)-1 ;
  coeffs = zeros(nseg,3,2) ;
  for k=1:nseg
    coeffs(k,:,:) = toPoly( breaks(k+1)-breaks(k), P0(:,k), P1(:,k), P2(:,k) ) ;
  end
  ppx = mkpp( breaks, coeffs(1,:,:) ) ;
  ppy = mkpp( breaks, coeffs(2,:,:) ) ;
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
function [P0, P1, P2] = LSQfixed( t, PNTS )

  [b0,b1,b2] = base( t ) ;

  A = [b0 ; b1 ; b2] ;
  Q = PNTS ;
  
  RES = A.' \ Q.' ; % risolvo ai minimi quadrati

  P0 = RES(1,:).' ;
  P1 = RES(2,:).' ;
  P2 = RES(3,:).' ;
end
