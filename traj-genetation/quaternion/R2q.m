function q=R2q(R)
    expC = R2exp(R);
    theta = norm(expC);
    if abs(theta) < 1e-8
        q = [1,0,0,0];
    else
        q = [cos(theta/2),expC(:)'/theta*sin(theta/2)];
    end
end