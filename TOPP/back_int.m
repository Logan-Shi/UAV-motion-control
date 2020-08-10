function [sb,sb_dot]=back_int(s,sdot,L,dt)
    sdot2=L(s,sdot);
    sb_dot=sdot-sdot2*dt;
    sb=s-sdot*dt;
end