function T = inv_thrust( rdot,rdot2,m,g,kd )
    % rdot ���ٶ�
    % rdot2 �߼��ٶ�
    % kd ճ������ϵ��
    gravity = [0; 0; -g];
    Fd = -kd * rdot;
    T=m*(rdot2-gravity-Fd);
end