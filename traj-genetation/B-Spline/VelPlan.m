function udot = VelPlan(v_u,a_u,Vmax,A_max)
Vsq_max = Vmax^2;%m^2/s^2
N = size(v_u,2);
u = linspace(0,1,N);
Max_target = -ones(1,N-2);
lb = zeros(1,N-2);
[ub,rho] = calc_vel_ub(Vsq_max,A_max,v_u,a_u,N-2);
[A,b,kapsq] = calc_acc_cstrts(v_u,a_u,u,A_max);
res = linprog(Max_target,A,b,[],[],lb,ub);
%check = A*res;
Vsq_u = [0;res;0];
udot = sqrt(Vsq_u./kapsq);
% chord_error = rho-sqrt(rho.^2-sqrt(Vsq_u(2:end-1)')*(t(2)-t(1))/2);
% figure()
% plot(u(2:end-1),chord_error)
% title('chord error')
% xlabel('u')
% ylabel('m')
end