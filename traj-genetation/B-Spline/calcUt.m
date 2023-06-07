function [ut,udott,uddott] = calcUt(t,u,udot)
%ODE���dudt=f(u)
f = @(x) udotInterU(x,u,udot);
ut = zeros(1,length(t));
ut(1) = 0.00001;%��ʼ��ֹ����һ���Ŷ�
udott = zeros(1,length(t));
h = t(2)-t(1);
for i = 1:length(ut)-1
    yi = ut(i);
    k1 = f(yi);
    k2 = f(yi+h/2*k1);
    k3 = f(yi+h/2*k2);
    k4 = f(yi+h*k3);
    ut(i+1) = min(1,yi+h/6*(k1+2*k2+2*k3+k4));
    udott(i+1) = f(ut(i+1));%ע���ʱudottĬ��0
end
uddott = FDMinter3(t,udott);
end

