function udot = backwardScan(u,udot,v_u,a_u)
udot(end) = 0;
for i = length(u):-1:2
    check = constrBack(udot(i-1),v_u(:,i-1),a_u(:,i-1),udot(i),u(2)-u(1));
    if max(check)>0
        [newudot,exitcond] = calcNewUdotBack(v_u(:,i-1),a_u(:,i-1),udot(i),u(2)-u(1));
        if exitcond<=0
            [newudot,newlastudot] = BisectionMethod(@(lastudot) calcNewUdotBack(v_u(:,i-1),a_u(:,i-1),lastudot,u(2)-u(1)),0,1,1e-3);
            udot(i) = newlastudot;
        end
        if newudot<udot(i-1)
            udot(i-1) = newudot;
        end
    end
end
end

function [newu,exitcond] = calcNewUdotBack(v,a,lastudot,du)
[newu,~,exitcond,~] = fmincon(@(x) -x,0,[],[],[],[],0,1,@(x) constrBack(x,v,a,lastudot,du));
end

function uddot = uddot(udoti,udotlasti,du)
uddot = (udoti^2 - udotlasti^2)/2/du;
end

function [c,ceq] = constrBack(udot,v,a,lastudot,du)
vmax = 3;%m/s
amax = 4;
c(1) = v(1)*udot - vmax;
c(2) = a(1)*udot^2+v(1)*uddot(lastudot,udot,du)-amax;

c(3) = v(2)*udot - vmax;
c(4) = a(2)*udot^2+v(2)*uddot(lastudot,udot,du)-amax;

c(5) = v(3)*udot - vmax;
c(6) = a(3)*udot^2+v(3)*uddot(lastudot,udot,du)-amax;
ceq = [];
end

function [newudot,newlastudot] = BisectionMethod(f,a,b,TOL)
while (b-a)/2 > TOL
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