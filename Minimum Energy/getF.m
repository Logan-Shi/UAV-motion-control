function F=getF(PI,Jd,h)
    % PI是角动量矢量
    % Jd是非标准惯量矩阵，是对角阵
    % h是时间步长
    TEMP=PI*h;
    J1=Jd(1,1);
    J2=Jd(2,2);
    J3=Jd(3,3);
    
    f=@(Fexp) fun(Fexp,J1,J2,J3,TEMP);
    FEXP=fsolve(f,[1;1;1]);
    S_FEXP=xev(FEXP);
    N_FEXP=norm(FEXP);
    F=eye(3)+S_FEXP./N_FEXP.*sin(N_FEXP)+S_FEXP*S_FEXP./(N_FEXP^2).*(1-cos(N_FEXP));
end

function error=fun(Fexp,J1,J2,J3,TEMP)
    S_Fexp=xev(Fexp);
    n_Fexp=norm(Fexp);
    Ftemp=eye(3)+S_Fexp./n_Fexp.*sin(n_Fexp)+S_Fexp*S_Fexp./(n_Fexp^2).*(1-cos(n_Fexp));
    temp=zeros(3,1);
    temp(1)=J2*Ftemp(3,2)-J3*Ftemp(2,3);
    temp(2)=J3*Ftemp(1,3)-J1*Ftemp(3,1);
    temp(3)=J1*Ftemp(2,1)-J2*Ftemp(1,2);
    error=TEMP-temp;
end