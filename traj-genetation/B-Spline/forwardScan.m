function udot = forwardScan(u,udot,v_u,a_u)
    for i = 2:length(u)
        [newudot,exitcond] = calcNewUdot(v_u(:,i),a_u(:,i),udot(i-1),u(2)-u(1));
        if exitcond<=0
            [newudot,newlastudot] = BisectionMethod(@(lastudot) calcNewUdot(v_u(:,i),a_u(:,i),lastudot,u(2)-u(1)),0,1,1e-3);
            udot(i-1) = newlastudot;
        end
        udot(i) = newudot;
    end
end

function [newu,exitcond] = calcNewUdot(v,a,lastudot,du)
    options = optimoptions('fmincon','Display','off');
    [newu,~,exitcond,~] = fmincon(@(x) -x,0,[],[],[],[],0,1,@(x) constr(x,v,a,lastudot,du),options);
%     [newu,~,exitcond,~] = ga(@(x) -x,1,[],[],[],[],0,1,@(x) constr(x,v,a,lastudot,du));
end    

function uddot = uddot(udot,last_udot,du)
    uddot = (udot^2 - last_udot^2)/2/du;
end

function [c,ceq] = constr(udot,v,a,lastudot,du)
    vmax = 3;%m/s
    amax = 4;
    c(1) = v(1)*udot - vmax;
    c(2) = a(1)*udot^2+v(1)*uddot(udot,lastudot,du)-amax;
    
    c(3) = v(2)*udot - vmax;
    c(4) = a(2)*udot^2+v(2)*uddot(udot,lastudot,du)-amax;
    
    c(5) = v(3)*udot - vmax;
    c(6) = a(3)*udot^2+v(3)*uddot(udot,lastudot,du)-amax;
    ceq = [];
end

function [newudot,newlastudot] = BisectionMethod(f,a,b,TOL)
exitflag = 0;
while (b-a)/2 > TOL || exitflag<=0
    c = (a+b)/2;
    [newudot,exitflag] = f(c);
    
    if exitflag<=0
        b = c;
    else
        a = c;
    end
    newlastudot = (a+b)/2;
end
end