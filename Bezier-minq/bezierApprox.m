function [P0v,T0v,P1v,T1v,maxSqDist,ibe] = bezierApprox( TNODES, PNTS, MxAllowSqD )

  if ( MxAllowSqD < 0 )
    error('Max. Allowed Square Distance should be >= 0');    
  end

  ibe = [1 size(PNTS,2)] ;
  uno = ones( 1, size(PNTS,1) ) ;

  while ( true )

    maxSqDist = 0 ;
    P0v = [] ;
    T0v = [] ;
    P1v = [] ;
    T1v = [] ;

    newibe = [] ;

    for k=1:length(ibe)-1
      P  = PNTS(:,ibe(k):ibe(k+1)) ;
      ti = TNODES(ibe(k):ibe(k+1)) ;
      ti = (ti-ti(1)) / (ti(end)-ti(1)) ;
      [P0,T0,P1,T1] = cubicMinqFixedExtrema(ti,P) ;
      P0v = [P0v P0] ;
      T0v = [T0v T0] ;
      P1v = [P1v P1] ;
      T1v = [T1v T1] ;
      ERR = uno * (cubic(ti,P0,T0,P1,T1) - P).^2 ;
      [sqDist,sqIndex] = max(sqrt(ERR(2:end-1))) ;
      sqDist = max(sqrt(ERR(2:end-1))) ;

      maxSqDist = max( maxSqDist, sqDist ) ;

      idx = sqIndex + ibe(k) ;
      if sqDist > MxAllowSqD && ibe(k+1)-ibe(k) > 7
        if sqIndex > 3 && idx < ibe(k+1) - 3
          newibe = [ newibe ibe(k) idx ] ;
        else
          idx    = floor((ibe(k+1)+ibe(k))/2) ;
          newibe = [ newibe ibe(k) idx ] ;
        end
      else
        newibe = [ newibe ibe(k) ] ;
      end
    end

    newibe = [ newibe ibe(end) ] ;

    if ( maxSqDist < MxAllowSqD ) ; break ; end ;
    if ( length(newibe) == length(ibe) ) ; break ; end ;

    ibe = newibe ;
  end
end
