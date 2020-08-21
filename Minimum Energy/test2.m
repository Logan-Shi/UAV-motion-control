    clear
    clc
    lambda=[-1;-1;-1;
            -10;-10;-10;
            -1;-1;-1;
            0;0;0];


m=1;
    g=9.8;
    J=diag([0.081,0.081,0.142]);
    Jd=0.5*trace(J)*eye(3)-J;
   
    X_start=zeros(3,1);
    P_start=zeros(3,1);
    R_start=eye(3);
    PI_start=zeros(3,1);
    
    X_end=zeros(3,1);
    P_end=zeros(3,1);
    R_end=eye(3);
    PI_end=zeros(3,1);
    
    T=10;
    h=1;
    N=T/h;
    
    
    lambda_X0=lambda(1:3);
    lambda_P0=lambda(4:6);
    lambda_R0=lambda(7:9);
    lambda_PI0=lambda(10:12);
    
    lambda_X_series=zeros(3,N+1);
    lambda_P_series=zeros(3,N+1);
    lambda_R_series=zeros(3,N+1);
    lambda_PI_series=zeros(3,N+1);
    
    lambda_X_series(:,1)=lambda_X0;
    lambda_P_series(:,1)=lambda_P0;
    lambda_R_series(:,1)=lambda_R0;
    lambda_PI_series(:,1)=lambda_PI0;
    
    X_series=zeros(3,N+1);
    P_series=zeros(3,N+1);
    R_series=zeros(3,3,N+1);
    PI_series=zeros(3,N+1);
    
    X_series(:,1)=X_start;
    P_series(:,1)=P_start;
    R_series(:,:,1)=R_start;
    PI_series(:,1)=PI_start;
    
    for k=1:N
        Xk=X_series(:,k);
        Pk=P_series(:,k);
        Rk=R_series(:,:,k);
        PIk=PI_series(:,k);
        lambda_Xk=lambda_X_series(:,k);
        lambda_Pk=lambda_P_series(:,k);
        lambda_Rk=lambda_R_series(:,k);
        lambda_PIk=lambda_PI_series(:,k);
        
        
        Fk=GET_F(PIk,Jd,h);
        
        
        
        
        Rnext=Rk*Fk;
        fnext=-transpose(lambda_Pk)*Rnext*[0;0;1];
        Mnext=-lambda_PIk;
        
        Xnext=Xk+h/m*Pk;
        Pnext=Pk-h*m*g*[0;0;1]+h*fnext*Rnext*[0;0;1];
        PInext=transpose(Fk)*PIk+h*Mnext;
        
        Fnext=GET_F(PInext,Jd,h);
        Bnext=(h*transpose(Fnext))/(trace(Fnext*Jd)*eye(3)-Fnext*Jd);
        
        A=zeros(12);
        A(1:3,1:3)=eye(3);
        A(4:6,1:3)=h/m*eye(3);
        A(4:6,4:6)=eye(3);
        A(7:9,7:9)=Fnext;
        A(10:12,7:9)=transpose(Bnext);
        A(10:12,10:12)=Fnext-transpose(Bnext)*xev((transpose(Fnext)*PInext));
        
        B=eye(12);
        B(7:9,4:6)=h*fnext*xev([0;0;1])*transpose(Rnext);
        
        lambda_k=[lambda_Xk;lambda_Pk;lambda_Rk;lambda_PIk];
        lambda_next=A\(B*lambda_k);
        
        X_series(:,k+1)=Xnext;
        P_series(:,k+1)=Pnext;
        R_series(:,:,k+1)=Rnext;
        PI_series(:,k+1)=PInext;
        
        lambda_X_series(:,k+1)=lambda_next(1:3);
        lambda_P_series(:,k+1)=lambda_next(4:6);
        lambda_R_series(:,k+1)=lambda_next(7:9);
        lambda_PI_series(:,k+1)=lambda_next(10:12);
        
    end
    
    X_end_cal=X_series(:,end);
    P_end_cal=P_series(:,end);
    R_end_cal=R_series(:,:,end);
    PI_end_cal=PI_series(:,end);
    
    norm_X=norm(X_end_cal-X_end);
    norm_P=norm(P_end_cal-P_end);
    norm_R=norm(R_end*transpose(R_end_cal),Inf);
    norm_PI=norm(PI_end_cal-PI_end);
    
    error=norm_X+norm_P+norm_R+norm_PI;