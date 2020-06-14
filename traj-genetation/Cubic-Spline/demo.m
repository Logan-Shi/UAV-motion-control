clear
clc
% r1=[0;0;0];
% R1=expm(hat(r1));
% r2=[0.5;-0.1;-0.3];
% R2=expm(hat(r2));
% r3=[1;0.2;-1];
% R3=expm(hat(r3));
% r4=[0;-0.2;-2.2];
% R4=expm(hat(r4));

R1=eye(3);
R2=rotx(45);
R3=roty(45)*R2;
R4=rotz(45)*R3;

R(:,:,1)=R1;
R(:,:,2)=R2;
R(:,:,3)=R3;
R(:,:,4)=R4;

T=[0;1;2;3];
omega0=[0.6,0.1,-0.3];
[a,b,c]=cubic_spline(T,R,omega0);
ti=linspace(0,3,1000);
Ri=cal_R(ti,T,R,a,b,c);

% 计算指数坐标r 3*1向量
ri=zeros(3,length(Ri));
for j=1:length(Ri)
    ri(:,j)=get_r(Ri(:,:,j));
end

% 三点中心差分计算r的导数
ri_dot=zeros(3,length(Ri));
for j=2:length(Ri)-1
    ri_dot(:,j)=(ri(:,j+1)-ri(:,j-1))/(ti(j+1)-ti(j-1));
end

hold on
plot(ti,ri(1,:))
plot(ti,ri(2,:))
plot(ti,ri(3,:))

figure
hold on
plot(ti,ri_dot(1,:))
plot(ti,ri_dot(2,:))
plot(ti,ri_dot(3,:))

% figure
% E=zeros(3,length(ti));
% for j=1:size(Ri,3)
%     E(:,j)=rotm2eul(Ri(:,:,j),'XYZ');
% end
% 
% hold on
% plot(ti,E(1,:))
% plot(ti,E(2,:))
% plot(ti,E(3,:))
% 
% rotm2eul(R(:,:,2),"XYZ")
% rotm2eul(R(:,:,3),"XYZ")
% rotm2eul(R(:,:,4),"XYZ")