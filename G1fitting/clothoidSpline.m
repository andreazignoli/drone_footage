function RES = clothoidSpline( P, T )

  npts = size( P, 2 ) ;
  RES  = zeros( npts-1, 6 ) ;

  x0     = P(1,1) ;
  y0     = P(2,1) ;
  theta0 = atan2( T(2,1), T(1,1) ) ;
  for i=2:npts
    x1         = P(1,i) ;
    y1         = P(2,i) ;
    theta1     = atan2( T(2,i), T(1,i) ) ;
    [k,dk,L]   = buildClothoid( x0, y0, theta0, x1, y1, theta1 ) ;
    RES(i-1,:) = [ x0, y0, theta0, k, dk, L ] ;
    x0         = x1 ;
    y0         = y1 ;
    theta0     = theta1 ;
  end

end
