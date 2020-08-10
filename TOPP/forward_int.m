function [sf,sf_dot]=forward_int(s,sdot,U,dt)
    sdot2=U(s,sdot);
    sf_dot=sdot+sdot2*dt;
    sf=s+sdot*dt;
end