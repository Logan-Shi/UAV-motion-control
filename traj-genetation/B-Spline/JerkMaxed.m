function Vsq_jerk = JerkMaxed(Vsq_u,v_u,a_u,j_u,t,kapsq,Jerk_max)
N = length(t);
u = linspace(0,1,N);
Max_target = -ones(1,N-2);
lb = zeros(1,N-2);
[A,b,qstar] = calc_jerk_cstrts(Vsq_u,v_u,a_u,j_u,u,Jerk_max,kapsq);
q = linprog(Max_target,A,b,[],[],lb,qstar(2:end-1));
check = A*q;
reso = kapsq(2:end-1).*q';
Vsq_jerk = [0;reso';0];
figure()
plot(u,sqrt(Vsq_u));
hold on
plot(u,sqrt(Vsq_jerk));
legend('without jerk contraint','with jerk constraint')
xlabel('u');
ylabel('V');
end