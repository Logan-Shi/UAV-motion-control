function udot = JerkMaxed(udot,v_u,a_u,j_u,Jerk_max)
N = size(v_u,2);
u = linspace(0,1,N);
kap_sq = zeros(N,1);
Vsq_u = zeros(N,1);
for i = 1:N
    kap_sq(i) = norm(v_u(:,i))^2;
    Vsq_u(i) = kap_sq(i)*udot(i)^2;
end
idex =find(Vsq_u(2:end-1)==0); 
Vsq_u(idex+1)=0.01;
Max_target = -ones(1,N-2);
lb = zeros(1,N-2);
[A,b,qstar] = calc_jerk_cstrts(Vsq_u,v_u,a_u,j_u,u,Jerk_max,kap_sq);
q = linprog(Max_target,A,b,[],[],lb,qstar(2:end-1));
%check = A*q;
reso = kap_sq(2:end-1).*q;
Vsq_jerk = [0;reso;0];
% hold on
udot = sqrt(Vsq_jerk./kap_sq);
end