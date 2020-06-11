function A = calc_acc_cstrts(p_u,v_u,a_u,u)
N = length(u)-2;
den = u(2) - u(1);
tmp = zeros(N+2,N,3);
kap_sq = zeros(1,N+2);
for i = 1:N+2
    kap_sq(i) = norm(v_u(:,i))^2;
end

vec_m = zeros(1,N);
vec_u = zeros(1,N-1);
vec_l = zeros(1,N-1);
for j = 1:3
    for i = 1:N
        vec_m(i) = a_u(j,i+1)/kap_sq(i+1);
    end
    for i = 1:N-1
        vec_u(i) = v_u(j,i+1)/kap_sq(i+2)/4/den;
        vec_l(i) = -v_u(j,i+2)/kap_sq(i+1)/4/den;
    end
    tmp(2:end-1,:,j) = diag(vec_m)+diag(vec_u,1)+diag(vec_l,-1);
    first = zeros(1,N);
    first(1) = v_u(j,2)^2/kap_sq(2)/abs(p_u(j,2))/2;
    last = zeros(1,N);
    last(end) = v_u(j,end-1)^2/kap_sq(end-1)/abs(p_u(j,end-1))/2;
    tmp(1,:,j) = first;
    tmp(end,:,j) = last;
end
A = [tmp(:,:,1);tmp(:,:,2);tmp(:,:,3)];
end

