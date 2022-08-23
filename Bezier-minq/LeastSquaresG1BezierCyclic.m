%
% Function that calculates the least square problems for a G1-smooth 
% cubic bezier curve. 
% It uses the initial guess given by BezierApprox to solve a linear
% system of equations resulting in Pk, Tk
%
function [P0,T0,P1,T1] = LeastSquaresG1BezierCyclic( TNODES, PNTS, ibe )
  %[Lv] = LengthBezierk [P0v,T0v,P1v,T1v,ibegin,iend]
  % to be used for a better approximation of the k bezier curves
  % for the moment we will use the polygonal length given by ti

  n = length(ibe)-1;
  M = sparse(2*n,2*n); 
  % M is the matrix of the linear system of the LQP

  Q = zeros(2*n,2);
  % Q is the vector of the PNTS for the system Mx=Q
  % where x is the vector of the optimized control points for the G1-smooth
  % piecewise bezier curve

  L = ones(n,1);
  W = ones(n,1);
  for k=1:n
    L(k) = TNODES(ibe(k+1))-TNODES(ibe(k)) ;
    %W(k) = ibe(k+1)-ibe(k)+1 ;
  end

  Pj = PNTS(:,ibe(n):ibe(n+1)) ;
  Lj = L(n) ; % salvo lunghezza
  tj = TNODES(ibe(n):ibe(n+1)) ;
  tj = (tj-tj(1))/(tj(end)-tj(1)) ; % riscalo in [0,1]
  wj = W(n) ;

  for k=1:n

    if k == n ; kp1 = 1 ; else kp1 = k+1 ; end ;
  
    Pi = Pj ;
    Li = Lj ;
    ti = tj ;
    ti = tj ;
    wi = wj ;

    Pj = PNTS(:,ibe(k):ibe(k+1)) ;
    Lj = L(k) ;
    tj = TNODES(ibe(k):ibe(k+1)) ;
    tj = (tj-tj(1))/(tj(end)-tj(1)) ; % riscalo in [0,1]
    wj = W(k) ;
    
    ID   = 2*k-1:2*k ;
    IDp1 = 2*kp1-1:2*kp1 ;

    % definition of the upper triangular matrix, first row
    M(ID,ID) = DIAG( ti, wi, Li, tj, wj, Lj ) ;

    % definition of the upper triangular matrix, second row
    M(ID,IDp1) = RIGHT( tj, wj, Lj ) ;
    M(IDp1,ID) = M(ID,IDp1) .' ;

    % definition of the points vector
    Q(ID,:) = RHS( ti, wi, Li, Pi, tj, wj, Lj, Pj ) ;
  end

  RES = M\Q ;

  P0 = [RES(1:2:end,1).' ; RES(1:2:end,2).'] ;
  T0 = [RES(2:2:end,1).' ; RES(2:2:end,2).'] ;
  
  P1 = [ P0(:,2:end) P0(:,1) ] ;
  T1 = [ T0(:,2:end) T0(:,1) ] ;
  
  T0 = (diag(L)*T0.').' ;
  T1 = (diag(L)*T1.').' ;

end

function D = DIAG( ti, wi, leni, tj, wj, lenj )
  [ bi0, ci0, bi1, ci1 ] = baseCubicBezier( ti ) ;
  [ bj0, cj0, bj1, cj1 ] = baseCubicBezier( tj ) ;
  D      = zeros(2,2) ;
  D(1,1) = sum(bi1.*bi1)*wi + sum(bj0.*bj0)*wj ;
  D(1,2) = sum(bi1.*ci1)*(wi*leni) + sum(bj0.*cj0)*(wj*lenj) ;
  D(2,1) = D(1,2) ;
  D(2,2) = sum(ci1.*ci1)*(wi*leni^2) + sum(cj0.*cj0)*(wj*lenj^2) ;
end

function R = RIGHT( tj, wj, lenj )
  [ bj0, cj0, bj1, cj1 ] = baseCubicBezier( tj ) ;
  R      = zeros(2,2) ;
  R(1,1) = sum(bj0.*bj1) * wj ;
  R(1,2) = sum(bj0.*cj1) * (wj*lenj) ;
  R(2,1) = sum(bj1.*cj0) * (wj*lenj) ;
  R(2,2) = sum(cj0.*cj1) * (wj*lenj^2) ;
end

function  Q = RHS( ti, wi, leni, Pi, tj, wj, lenj, Pj )
  [ bi0, ci0, bi1, ci1 ] = baseCubicBezier( ti ) ;
  [ bj0, cj0, bj1, cj1 ] = baseCubicBezier( tj ) ;

  Q      = zeros(2,size(Pi,1)) ;
  Q(1,:) = wi * (bi1*Pi.') + wj * (bj0*Pj.') ;
  Q(2,:) = (wi*leni) * (ci1*Pi.') + (wj*lenj) * (cj0*Pj.') ;
end
