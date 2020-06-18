function omg=omegaQ(x)
    x = x(:)';
    omg = [
        x;
        -x(2), x(1), x(4), -x(3);
        -x(3), -x(4), x(1), x(2);
        -x(4), x(3), -x(2), x(1);
    ];
end