function Vsq_u = VelPlan(t,v_u,a_u)
Vsq_max = 4;%m^2/s^2
Track_err_max = 0.1;%m
Acc_x_max = 0.3;%m/s^2
Acc_y_max = 0.3;%m/s^2
Acc_z_max = 0.3;%m/s^2
N = length(t);
Max_target = -ones(1,N-2);
lb = zeros(1,N-2);
ub = calc_vel_ub(Track_err_max,t,Vsq_max,v_u,a_u);
b = [Acc_x_max*ones(1,N-2),Acc_y_max*ones(1,N-2),Acc_z_max*ones(1,N-2)];
A = calc_acc_cstrts(v_u,a_u,t);
res = linprog(Max_target,[A;-A],[b,b],[],[],lb,ub);
check = A*res;
Vsq_u = [0;res;0];
end