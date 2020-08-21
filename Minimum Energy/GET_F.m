function F=GET_F(PI,Jd,h)
    % PI是角动量矢量
    % Jd是非标准惯量矩阵，是对角阵
    % h是时间步长
    TEMP=PI*h;
    J1=Jd(1,1);
    J2=Jd(2,2);
    J3=Jd(3,3);
    f=@(Fs) fun(Fs,J1,J2,J3,TEMP);
    Fs0=[0;0;0;0;0;0;0;0;0];
    Fs=fsolve(f,Fs0);
    
    F=[Fs(1),Fs(4),Fs(7);
       Fs(2),Fs(5),Fs(8);
       Fs(3),Fs(6),Fs(9)];
end

function e=fun(Fs,J1,J2,J3,temp)
    e(1)=J2*Fs(6)-J3*Fs(8)-temp(1);
    e(2)=J3*Fs(7)-J1*Fs(3)-temp(2);
    e(3)=J1*Fs(2)-J2*Fs(4)-temp(3);
    e(4)=Fs(1)^2+Fs(2)^2+Fs(3)^2-1;
    e(5)=Fs(4)^2+Fs(5)^2+Fs(6)^2-1;
    e(6)=Fs(7)^2+Fs(8)^2+Fs(9)^2-1;
    e(7)=Fs(1)*Fs(4)+Fs(2)*Fs(5)+Fs(3)*Fs(6);
    e(8)=Fs(1)*Fs(7)+Fs(2)*Fs(8)+Fs(3)*Fs(9);
    e(9)=Fs(7)*Fs(4)+Fs(8)*Fs(5)+Fs(9)*Fs(6);
end
