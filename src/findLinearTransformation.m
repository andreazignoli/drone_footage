function [T,A] = findLinearTransformation(u,v)
% finds a matrix T (nP X nP) such that T * u(:,i) = v(:,i)
% u(:,i) and v(:,i) are n-dim col vectors; the amount of col vectors in u and v must match (and are equal to nP)
%
    if any(size(u) ~= size(v))
        error('findLinearTransform:u','u and v must be the same shape and size n-dim vectors');
    end
    [n,nP] = size(u); % n -> dimensionality; nP -> number of points to be transformed
    if nP > n % if the number of points to be transform exceeds the dimensionality of points
        I = eye(nP);
        u = [u;I((n+1):nP,1:nP)]; % then fill up the points to be transformed with the identity matrix
        v = [v;I((n+1):nP,1:nP)]; % as well as the transformed points
        [n,nP] = size(u);
    end
    A = zeros(nP*n,n*n);
    for k = 1:nP
        for i = ((k-1)*n+1):(k*n)
            A(i,mod((((i-1)*n+1):(i*n))-1,n*n) + 1) = u(:,k)';
        end
    end
    v = v(:);
    T = reshape(A\v, n, n).';
end