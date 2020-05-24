function A = calc_acc_cstrts(v_u,a_u,t)
N = length(t)-2;
Ts = t(2) - t(1);
A = zeros(3*N,N);

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
        vec_u(i) = v_u(j,i+1)/kap_sq(i+2)/4/Ts;
        vec_l(i) = -v_u(j,i+2)/kap_sq(i+1)/4/Ts;
    end
    A(((j-1)*N+1):j*N,1:N) = diag(vec_m)+diag(vec_u,1)+diag(vec_l,-1);
end
end

