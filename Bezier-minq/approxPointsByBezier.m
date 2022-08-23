function [P0v,T0v,P1v,T1v,maxSqDist,ibe] = approxPointsByBezier( TNODES, PNTS, MxAllowSqD )

  if ( MxAllowSqD < 0 )
    error('Max. Allowed Square Distance should be >= 0');    
  end

  N   = size(PNTS,2) ; % totale punti
  
  ibe = [1] ;
  maxSqDist = 0 ;

  nseg = 1 ;
  P0v  = [] ;
  T0v  = [] ;
  P1v  = [] ;
  T1v  = [] ;

  while ( true )

    % costruzione segmento nseg (il primo � interpolante)
    inext = ibe(end)+3 ;
    if ( inext >= N )
      ibe(end) = N ;
      P  = PNTS(:,ibe(end-1):ibe(end)) ; % estraggo puntio candidati
      ti = TNODES(ibe(end-1):ibe(end)) ;
      ti = (ti-ti(1)) / (ti(end)-ti(1)) ;
      [P0,T0,P1,T1] = cubicBezier('lsq',ti,P,0) ; %% costruisco spline
      sqDist = cubicBezier('error',ti,P0,T0,P1,T1,P) ; %% calcolo errore
      sqDists = max(sqDists,sqDist) ; % salva la soluzione
      P0v(:,end) = P0 ;
      T0v(:,end) = T0 ;
      P1v(:,end) = P1 ;
      T1v(:,end) = T1 ;
      break ;
    end

    sqDists = 1E10 ;
    while inext < N
      inext = inext + 1 ; %% aggiungo punto
      P  = PNTS(:,ibe(end):inext) ; % estraggo puntio candidati
      ti = TNODES(ibe(end):inext) ;
      ti = (ti-ti(1)) / (ti(end)-ti(1)) ;
      [P0,T0,P1,T1] = cubicBezier('lsq',ti,P,0) ; %% costruisco spline
      sqDist = cubicBezier('error',ti,P0,T0,P1,T1,P) ; %% calcolo errore
      if sqDist < max(MxAllowSqD,sqDists) % salva la soluzione
        P0s      = P0 ;
        P1s      = P1 ;
        T0s      = T0 ;
        T1s      = T1 ;
        is       = inext ;
        sqDists  = sqDist ;
      else % sta peggiorando, interrompo!
        break ;
      end
    end
    
    if sqDists > MxAllowSqD
      fprintf( 1, 'Exceding error at segment N.%d S=[%f,%f] ERR = %f\n', ...
                   length(ibe), TNODES(ibe(end)), TNODES(is), sqDists ) ;
    end

    % aggiunge spline alla lista
    P0v = [ P0v P0s ] ;
    T0v = [ T0v T0s ] ;
    P1v = [ P1v P1s ] ;
    T1v = [ T1v T1s ] ;
    ibe = [ ibe is  ] ;
    maxSqDist = max( maxSqDist, sqDists ) ;
  end
   
end