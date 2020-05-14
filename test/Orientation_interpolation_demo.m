clear all;
%input
pointNum = 6;
k = 2;
P = [9.036145, 21.084337, 37.607573, 51.893287, 61.187608;
    51.779661, 70.084746, 50.254237, 69.745763, 49.576271;
        10         -10           20        -20       30];
angle = (1:pointNum)*pi/pointNum;
R = zeros(3,3,pointNum);
for i=1:pointNum
    R(:,:,i) = rotZ(angle(i))*rotY(angle(i))*rotX(angle(i));
end

% function demo
Tf = 10;
u = linspace(0,Tf,100);
[y_u,yd_u] = OriInter(R,3,u);

plot(u,y_u,u,yd_u)
legend('yaw','yawd')
title('yaw desired')