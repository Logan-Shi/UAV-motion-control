clear
clc

s_start=0;
s_end=1;

dt=0.001;

TOL=0.001;

% ��ʼ��
S=[0,0];
i=0;

si=0;
si_dot=0;

sdot_max=10;

% ��������F
F=[s_end,0];
j=1;
while(true)
    if(Lower(F(j,1),F(j,2)) > Upper(F(j,1),F(j,2))) || norm(F(j,2))>sdot_max
        break;
    end
    [F(j+1,1),F(j+1,2)]=back_int(F(j,1),F(j,2),@Lower,dt);
    j=j+1;
end

hit_F=0;
% ��ѭ��
while(true)
    si=S(end,1);
    si_dot=S(end,2);
    A=[si,si_dot];
    
    k=1;
    % ǰ����ֵ�ѭ��
    while(true)
        % �����F�ཻ����������ɣ�������ѭ��
        if check(A,F,TOL) == 1
            hit_F=1;
            break;
        end
        
        % ��ײMVC������ת����������
        if(Lower(A(k,1),A(k,2)) > Upper(A(k,1),A(k,2))) || norm(A(k,2))>sdot_max
            slim=A(k,1);
            slimdot=A(k,2);
            break;
        end
        
        [A(k+1,1),A(k+1,2)]=forward_int(A(k,1),A(k,2),@Upper,dt);
        k=k+1;
    end
    
    if hit_F==1
        S(i+1,:)=A(end,:);
        i=i+1;
        break;
    end
    
    % ִ�ж����������õ����е�
    [stan,standot]=bisection(slim,slimdot,@Lower,@Upper,dt,TOL);
    
    %�����е�������
    n=1;
    Tan=[stan,standot];
    while(true)
        if check(A,Tan,TOL)==1
            S(i+1,:)=Tan(end,:);
            i=i+1;
            break;
        end
        [Tan(n+1,1),Tan(n+1,2)]=back_int(Tan(j,1),Tan(j,2),@Lower,dt);
        n=n+1;
    end
    S(i+1,:)=[stan,standot];
    i=i+1;
end