function Ri=cal_R_v2(ti,T,R,a,b,c)
    T_temp=T(2:end-1);
    m=length(ti);
    for i=1:m
        t=ti(i);
        K=1;
        for j=1:length(T_temp)
            K(t>=T_temp(j))=j+1;
        end
        
        tau=(t-T(K))/(T(K+1)-T(K));
        Ri(:,:,i)=R(:,:,K)*my_exp(a(:,K)*tau^3+b(:,K)*tau^2+c(:,K)*tau);
    end
end