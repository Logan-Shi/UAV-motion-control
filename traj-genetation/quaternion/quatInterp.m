function R=quatInterp(ts, Rs, newt, omega0, alpha0)
    if nargin <=4
        alpha0 = zeros(3,1);
        if nargin == 3
            omega0 = zeros(3,1);
        end
    end

    q = R2q(Rs(:,:,1).'*Rs(:,:,2))';

    c(:,1) = 1/2*[0;omega0(:)];
    b(:,1) = 1/4*([0;alpha0(:)]-2*omegaQ(c(:,1))*c(:,1));
    a(:,1) = q - b(:,1) - c(:,1) - [1;0;0;0];

    for k = 2:(length(ts)-1)
        s = R2q(Rs(:,:,k).'*Rs(:,:,k+1))';
        t = 3*a(:,k-1) + 2*b(:,k-1) + c(:,k-1);
        u = 6*a(:,k-1) + 2*b(:,k-1);
        c(:,k) = omegaQ(s)*t;
        b(:,k) = 1/2*(omegaQ(t)*t+omegaQ(s)*u-omegaQ(c(:,k))*c(:,k));
        a(:,k) = s-b(:,k)-c(:,1)-[1;0;0;0];
    end
    
    R = zeros(3,3,length(newt));
    for k = 1:length(newt)
        t = newt(k);
        if t < ts(1)
            R(:,:,k) = nan(3,3);
        elseif t > ts(end)
            R(:,:,k) = nan(3,3);
        else
            id = find((ts-t)>=0,1)-1;
            if t == ts(1)
                id = 1;
            end
            tau = (t-ts(id))/(ts(id+1)-ts(id));
            x = a(:,id)*tau^3 + b(:,id)*tau^2 + c(:,id)*tau + [1;0;0;0];
            R(:,:,k) = 1/(x.'*x)*Rs(:,:,id)*q2R(x);
        end
    end

end