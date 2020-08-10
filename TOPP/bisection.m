function [stan,standot]=bisection(slim,slimdot,L,U,dt,TOL)
    sdot_high=slimdot;
    sdot_low=0;
    done=0;
    while(true)
        sdot_test=(sdot_high+sdot_low)/2;
        Test=[slim,sdot_test];
        m=1;
        while(true)
            % ºÏ≤‚”ÎMVCœ‡«–
            if tangent(Test,L,U,dt,TOL)   
                done=1;
                stan=Test(end,1);
                standot=Test(end,2);
                break;
            end
            % ºÏ≤‚”ÎMVC≈ˆ◊≤
            if penetrate(Test,L,U,dt,TOL)
                sdot_high=sdot_test;
                break;
            end
            % ºÏ≤‚”Îsdot=0≈ˆ◊≤
            if norm(Test(end,2)) < TOL  
                sdot_low=sdot_test;
                break;
            end
            Test(m+1,:)=forward_int(Test(m,1),Test(m,2),L,dt);
        end
        
        if done==1
            break;
        end
    end
end

function tangented=tangent(Test,L,U,ds,TOL)
    e1=U(Test(end,1),Test(end,2))-L(Test(end,1),Test(end,2));
    [s_next,sdot_next]=forward_int(Test(end,1),Test(end,2),U,ds);
    e2=L(s_next,sdot_next)-U(s_next,sdot_next);
    if e1>0 && e1<TOL && e2>0 && e2< TOL
        tangented=1;
    else
        tangented=0;
    end
end

function penetrated=penetrate(Test,L,U,ds,TOL)
    e1=U(Test(end,1),Test(end,2))-L(Test(end,1),Test(end,2));
    [s_next,sdot_next]=forward_int(Test(end,1),Test(end,2),U,ds);
    e2=L(s_next,sdot_next)-U(s_next,sdot_next);
    if e1>0 && e1<TOL && e2>0 && e2< TOL
        penetrated=0;
    else
        penetrated=1;
    end
end