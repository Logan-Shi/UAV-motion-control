function [A,b,kap_sq] = calc_acc_cstrts(u,v_u,a_u,Amax)
N = length(u);
du = u(2) - u(1);
kap_sq = zeros(N,1);
for i = 1:N
    kap_sq(i) = norm(v_u(:,i))^2;
end

tmp4j = zeros(2*N+2,N-2,3);
for j = 1:3
%     tmp4pos = zeros(N-2,N);
%     for i = 2:N-1
%         tmp4pos(i-1,i-1:i+1) = [-v_u(j,i)/kap_sq(i-1)/4/du,a_u(j,i)/kap_sq(i),v_u(j,i)/kap_sq(i+1)/4/du];
%     end
%     tmp4neg = zeros(N-2,N);
%     for i = 2:N-1
%         tmp4neg(i-1,i-1:i+1) = -[-v_u(j,i)/kap_sq(i-1)/4/du,a_u(j,i)/kap_sq(i),v_u(j,i)/kap_sq(i+1)/4/du];
%     end
    tmp4pos = zeros(N-1,N);
    for i = 1:N-1
        tmp4pos(i,i:i+1) = [a_u(j,i)/kap_sq(i)-v_u(j,i)/4/kap_sq(i)/du,v_u(j,i)/kap_sq(i+1)/4/du];
    end
    tmp4neg = zeros(N-1,N);
    for i = 1:N-1
        tmp4neg(i,i:i+1) = -[a_u(j,i)/kap_sq(i)-v_u(j,i)/4/kap_sq(i)/du,v_u(j,i)/kap_sq(i+1)/4/du];
    end
    tmp4bnd = zeros(4,N-2);
    tmp4j(:,:,j) = [tmp4pos(:,2:end-1);tmp4neg(:,2:end-1);tmp4bnd];
end
A = [tmp4j(:,:,1);tmp4j(:,:,2);tmp4j(:,:,3)];
b = zeros(6*N+6,1)+Amax;
end