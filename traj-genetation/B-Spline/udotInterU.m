function udotu = udotInterU(x,u,udot)
if length(x)<=1
    if x<0 || x>1
        udotu = 0;
    else
        udotu = spline(u,udot,x);
    end
else
    udotu = spline(u,udot,x);
end

