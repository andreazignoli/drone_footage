%
%
function [maxDist,aveDist] = PiecewiseBezierError( TNODES, PNTS, ibe, P0, T0, P1, T1 )

  n   = length(ibe)-1;
  uno = ones(1,size(PNTS,1)) ;

  % calcolo errore
  maxDist = 0 ;
  accDist = [] ;
  for k=1:n
    P       = PNTS(:,ibe(k):ibe(k+1)) ;
    tj      = TNODES(ibe(k):ibe(k+1)) ;
    tj      = (tj-tj(1))/(tj(end)-tj(1)) ;
    ERR     = sqrt( uno * (cubicBezier( 'eval',tj,P0(:,k),T0(:,k),P1(:,k),T1(:,k)) - P).^2 ) ;
    maxDist = max( maxDist, max(ERR) ) ;
    accDist = [ accDist ERR ] ;
  end
  aveDist = sum( accDist ) / length(accDist) ;
end
