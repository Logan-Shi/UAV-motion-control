function omegadot = angular_acceleration(rotorspeeds, omega, I, L, b, k)
%   ��������ϵ�еĽǼ��ٶ�
    tau = torques(rotorspeeds, L, b, k);
    omegadot = I \ (tau - cross(omega, I * omega));
end