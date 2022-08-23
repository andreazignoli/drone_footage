function [d_traj, theta_traj, x_traj, y_traj] = fit_xy(X,Y,par_bezier)

% (E. Bertolazzi, DII-Unitn)

Xd = X;
Yd = Y;

PNTS = [Xd,Yd].';

% Node parametrisation

TNODES    = zeros(1,size(PNTS,2)) ;
TNODES(1) = 0 ;
for k=2:size(PNTS,2)
    TNODES(k) = TNODES(k-1) + norm( PNTS(:,k) - PNTS(:,k-1) ) ;
end

% clustering

[P0v,T0v,P1v,T1v,MxSqD,ibe] = approxPointsByBezier(TNODES,PNTS,par_bezier);

N = length(ibe)-1;

colors  = { '-r', '-b' } ;
colors1 = { ':r', ':b' } ;

% min least square
[P0,T0,P1,T1]     = LeastSquaresG1Bezier( TNODES, PNTS, ibe,'begin_free', 'end_free' ) ;

% average error
[maxDist,aveDist] = PiecewiseBezierError( TNODES, PNTS, ibe, P0, T0, P1, T1 ) ;
maxDist
aveDist

% polynomial format (MATLAB)

nseg        = size(P0,2) ;
tt          = TNODES(ibe) ;
[ppx,ppy]   = cubicBezier( 'pp', tt, P0, T0, P1, T1 ) ; % [0:1:nseg]

% points and tangent lines
P   = [ P0 P1(:,end) ] ;
T   = [ T0 T1(:,end) ] ;

% clothoids spline
CSP         = clothoidSpline( P, T ) ;
theta0      = CSP(1,3) ;

% plot clothoids segments
nseg        = size( CSP, 1 ) ;
npts        = 100 ;
distance    = 0;
x_traj      = [];
y_traj      = [];
d_traj      = [];
theta_traj  = [];
x_traj2     = [];
y_traj2     = [];
d_traj2     = [];
t           = 0:0.01:1;

for k=1:nseg
    
    % clothoids
    x0          = CSP(k,1) ;
    y0          = CSP(k,2) ;
    theta0      = CSP(k,3) ;
    kappa       = CSP(k,4) ;
    dkappa      = CSP(k,5) ;
    L           = CSP(k,6) ;
    
    tvec    = [0:L/npts:L] ;
    [iC,iS] = intCS( 1, dkappa, kappa, theta0, tvec );
    
    tmp        = atan2(diff(iS), diff(iC));
    tmp(tmp<pi) = tmp(tmp<pi) + 2*pi;
    if theta0 < pi
        theta0 = theta0 + 2*pi;
    end
    
    theta_traj = [theta_traj, theta0, tmp];
    
    % plot( x0+iC, y0+iS, colors{mod(k,2)+1}, 'LineWidth', 1.5);
    % text( mean(x0+iC), mean(y0+iS), num2str(k)) ;
    
    % clothoids
    x_traj = [x_traj, x0+iC];
    y_traj = [y_traj, y0+iS];
    d_traj = [d_traj, tvec + distance];
    
    % Bezier
    ttt     = tt(k) + t.* (tt(k+1)-tt(k));
    x_traj2 = [x_traj2, ppval(ttt,ppx)];
    y_traj2 = [y_traj2, ppval(ttt,ppy)];
    d_traj2 = [d_traj2, ttt];
    
    % update final distance
    distance = distance + L;
    
end

EOF = 1;


