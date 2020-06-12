function ut = trajInter(t,Vsq_u,v_u,a_u)
N = length(t);
u = linspace(0,1,N);
Ts = t(2)-t(1);
ut = zeros(1,N);
ut(1) = u(2)-u(1);
for i = 1:N-1
    Vsqi = pchip(u,Vsq_u,ut(i));
    V = sqrt(Vsqi);
    dC = v_u(:,i);
    ddC = a_u(:,i);
    BB = V*Ts/norm(v_u(:,i));
    AA = -V^2*Ts^2*dot(dC,ddC)/2/(norm(dC))^4;
    ut(i+1) = min(1,ut(i)+AA+BB);
end
figure()
plot(t,ut)
title('u(t)')
xlabel('time,s')
ylabel('param u')
end