function omega = eulerdot2omega(thetadot, euler)
%-----------------------------------------------------
%  Parameters :
%  thetadot     ŷ���Ǳ仯��
%  attitude     ŷ����
%  omega        ��������ϵ�Ľ��ٶ�
%----------------------------------------------------
phi = euler(1);
theta = euler(2);
psi = euler(3);
JB = [1  0           -sin(theta);
         0  cos(phi)    cos(theta)*sin(phi);
         0  -sin(phi)   cos(theta)*cos(phi)];
omega=JB * thetadot; 
end