function omega=rdot2omega(r,r_dot)
    nr=norm(r);
    omega=(eye(3)-(1-cos(nr)/(nr)^2)*hat(r)+(nr-sin(nr))/(nr)^3*(hat(r))^2)*r_dot;
end