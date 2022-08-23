%
% Function that calculates the least square problems for a G1-smooth 
% cubic bezier curve for an open path. 
% It uses the initial guess given by BezierApprox to solve a linear
% system of equations resulting in Pk, Tk
%
function [P0,T0,P1,T1] = LeastSquaresG1Bezier( TNODES, PNTS, ibe, varargin )

  OPTIONS = LeastSquaresG1BezierOptions(varargin{:})

  begin_fixed = ~ OPTIONS . begin_free ;
  end_fixed   = ~ OPTIONS . end_free ;

  n = length(ibe)-1;
  M = sparse(2*(n+1),2*(n+1)); 
  % M is the matrix of the linear system of the LQP

  Q = zeros(2*(n+1),2);
  % Q is the vector of the PNTS for the system Mx=Q
  % where x is the vector of the optimized control points for the G1-smooth
  % piecewise bezier curve

  L = ones(n,1);
  W = ones(n,1);
  for k=1:n
    L(k) = TNODES(ibe(k+1))-TNODES(ibe(k)) ;
    W(k) = ibe(k+1)-ibe(k)+1 ;
  end

  Pj = PNTS(:,ibe(1):ibe(2)) ;
  Lj = L(1) ; % salvo lunghezza
  tj = TNODES(ibe(1):ibe(2)) ;
  tj = (tj-tj(1))/(tj(end)-tj(1)) ; % riscalo in [0,1]
  wj = W(1) ;

  % definition of the upper triangular matrix, first row
  [ bj0, cj0, bj1, cj1 ] = cubicBezier( 'base', tj ) ;
  
  % first row block
  ID   = 1:2 ;
  IDp1 = 3:4 ;
  
  R21 = sum(bj1.*cj0) * (wj*Lj) ;
  R22 = sum(cj0.*cj1) * (wj*Lj^2) ;
  D22 = sum(cj0.*cj0) * (wj*Lj^2) ;

  if begin_fixed
    Pj         = Pj - PNTS(:,1)*bj0 ;
    M(ID,ID)   = [ 1 0 ; 0   D22 ] ;
    M(ID,IDp1) = [ 0 0 ; R21 R22 ] ;
    Q(1,:)     = PNTS(:,1).' ;
  else
    R12 = sum(bj0.*cj1) * (wj*Lj) ;
    R11 = sum(bj0.*bj1) * wj ;
    D11 = sum(bj0.*bj0) * wj ;
    D12 = sum(bj0.*cj0) * (wj*Lj) ;
    M(ID,ID)   = [ D11 D12 ; D12 D22 ] ;
    M(ID,IDp1) = [ R11 R12 ; R21 R22 ];
    Q(1,:)     = wj * (bj0*Pj.') ;
  end
  Q(2,:)     = (wj*Lj) * (cj0*Pj.') ;
  M(IDp1,ID) = M(ID,IDp1) .' ;
  
  for k=2:n
    kp1 = k+1 ;

    Pi = Pj ;
    Li = Lj ;
    ti = tj ;
    ti = tj ;
    wi = wj ;
    
    bi0 = bj0 ;
    ci0 = cj0 ;
    bi1 = bj1 ;
    ci1 = cj1 ;

    Pj = PNTS(:,ibe(k):ibe(kp1)) ;
    Lj = L(k) ;
    tj = TNODES(ibe(k):ibe(kp1)) ;
    tj = (tj-tj(1))/(tj(end)-tj(1)) ; % riscalo in [0,1]
    wj = W(k) ;
    
    [ bj0, cj0, bj1, cj1 ] = cubicBezier( 'base', tj ) ;

    D11 = sum(bi1.*bi1)*wi + sum(bj0.*bj0)*wj ;
    D12 = sum(bi1.*ci1)*(wi*Li)   + sum(bj0.*cj0)*(wj*Lj) ;
    D22 = sum(ci1.*ci1)*(wi*Li^2) + sum(cj0.*cj0)*(wj*Lj^2) ;

    ID   = 2*k-1:2*k ;
    IDp1 = 2*kp1-1:2*kp1 ;

    % definition of the upper triangular matrix
    M(ID,ID) = [ D11 D12 ; D12 D22] ;

    % definition of the upper triangular matrix
    R12 = sum(bj0.*cj1) * (wj*Lj) ;
    R22 = sum(cj0.*cj1) * (wj*Lj^2) ; 
    if k == n & end_fixed
      M(ID,IDp1) = [ 0 R12 ; 0 R22 ] ;
      Pj         = Pj - PNTS(:,end)*bj1 ;
    else      
      R11        = sum(bj0.*bj1) * wj ;
      R21        = sum(bj1.*cj0) * (wj*Lj) ;
      M(ID,IDp1) = [ R11 R12 ; R21 R22 ] ;
    end
    M(IDp1,ID) = M(ID,IDp1) .' ;

    % definition of the points vector
    Q(ID,:) = [ wi     * (bi1*Pi.') + wj      * (bj0*Pj.') ;
               (wi*Li) * (ci1*Pi.') + (wj*Lj) * (cj0*Pj.')] ;
  end

  Pi = Pj ;
  Li = Lj ;
  ti = tj ;
  ti = tj ;
  wi = wj ;

  bi0 = bj0 ;
  ci0 = cj0 ;
  bi1 = bj1 ;
  ci1 = cj1 ;

  % last row block
  ID   = 2*n+1:2*n+2 ;
  IDm1 = 2*n-1:2*n ;

  % definition of the upper triangular matrix, last row
  D22 = sum(ci1.*ci1)*(wi*Li.^2) ;
  if end_fixed
    M(ID,ID)   = [ 1 0 ; 0   D22 ] ;
    Q(ID(1),:) = PNTS(:,end).' ;
  else
    R12 = sum(bj0.*cj1) * (wj*Lj) ;
    R11 = sum(bj0.*bj1) * wj ;
    D11 = sum(bi1.*bi1) * wi ;
    D12 = sum(bi1.*ci1) * (wi*Li) ;
    M(ID,ID)   = [ D11 D12 ; D12 D22 ] ;
    Q(ID(1),:) = wi * (bi1*Pi.') ;
  end
  Q(ID(2),:) = (wi*Li) * (ci1*Pi.') ;
  
  RES = M\Q ;

  P0 = [RES(1:2:end,1).' ; RES(1:2:end,2).'] ;
  T0 = [RES(2:2:end,1).' ; RES(2:2:end,2).'] ;

  P1 = P0(:,2:end) ;
  T1 = T0(:,2:end) ;

  P0 = P0(:,1:end-1) ;
  T0 = T0(:,1:end-1) ;

  T0 = (diag(L)*T0.').' ;
  T1 = (diag(L)*T1.').' ;

end

function OPTIONS = LeastSquaresG1BezierOptions(varargin)

  OPTIONS . begin_free = 0 ;
  OPTIONS . end_free   = 0 ;

  Names = fieldnames(OPTIONS) ;

  % OPTIONS = PSOSET(OLDOPTS,'PARAM1',VALUE1,...
  for i=1:nargin
    f = varargin{i} ;
    if ~ischar(f)
      error('MATLAB:LeastSquaresG1BezierOptions:NoPropName','Expected argument %d to be a string property name.', i);
    end
    if any(strcmp(f,Names))
      OPTIONS.(f) = 1 ;
    else
      error('MATLAB:LeastSquaresG1BezierOptions:InvalidPropName','Unrecognized property name ''%s''.', f);
    end
  end  

end
