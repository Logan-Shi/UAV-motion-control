function Vsq_u = VelPlan(t,p_u,v_u,a_u)
Vsq_max = 5;%m^2/s^2
Track_err_max = 0.05;%m
Acc_x_max = 2;%m/s^2
Acc_y_max = 2;%m/s^2
Acc_z_max = 2;%m/s^2
N = length(t);
u = linspace(0,1,N);
Max_target = -ones(1,N-2);
lb = zeros(1,N-2);
ub = calc_vel_ub(Track_err_max,u,Vsq_max,v_u,a_u);
b = [Acc_x_max*ones(1,N),Acc_y_max*ones(1,N),Acc_z_max*ones(1,N)];
A = calc_acc_cstrts(p_u,v_u,a_u,u);
res = linprog(Max_target,[A;-A],[b,b],[],[],lb,ub);
check = A*res;
Vsq_u = [0;res;0];
end