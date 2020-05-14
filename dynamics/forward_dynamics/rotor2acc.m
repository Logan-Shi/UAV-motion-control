function a = rotor2acc(rotorspeeds, attitude, rdot, m, g, k, kd)
    % author: LZY
    % last edited: ZRT
    % change to the file:
    % ԭ�ļ��� acceleration, Ϊ��ֹ����, ����Ϊ rotor2acc
    % ֱ��������ת����attitude������ʹ��ŷ����
    
    %   rotorspeeds �ĸ������ת��
    %   attitude λ�� �任����
    %   rdot ���ٶ�ʸ�� ճ������ ���������ٶȳ�����
    gravity = [0; 0; -g];
    R = (attitude);
    T = R * thrust(rotorspeeds, k);
    Fd = -kd * rdot;
    a = gravity + 1 / m * T + Fd;
end
