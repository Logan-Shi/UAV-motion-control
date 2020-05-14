function thetadot = omega2eulerdot(omega, euler)

%  ����ŷ���ǵı仯��      
%-----------------------------------------------------
%  Parameters :
%       *.  thetadot ----- ŷ���Ǳ仯��
%       *.  attitude ----- ŷ����
%       *.  omega    ----- ���ٶ�
%----------------------------------------------------
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    if abs(theta-pi/2) > 1e-4
        thetadot = [1,0,          -sin(theta);
                    0,cos(phi)    cos(theta)*sin(phi);
                    0,-sin(phi)   cos(theta)*cos(phi)] \ omega ;
    else
        thetadot = omega;
    end
end

