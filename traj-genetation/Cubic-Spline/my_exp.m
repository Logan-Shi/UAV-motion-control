function A=my_exp(r)
    nr=norm(r);
    if nr<eps
        A=eye(3);
    else
        R=hat(r);
        A=eye(3)+sin(nr)/nr*R+(1-cos(nr))/nr^2*R*R;
    end
end