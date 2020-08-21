function [L,U] = feasibiltyCheck(u,udot,v_u,a_u,capibity)
A = capibity(2);
Lxyz = zeros(3,1);
Uxyz = zeros(3,1);
ux = linspace(0,1,size(a_u,2));
for i = 1:3
mintemp = (A-spline(ux,a_u(i,:),u)*udot.^2)/spline(ux,v_u(i,:),u);
maxtemp = (-A-spline(ux,a_u(i,:),u)*udot.^2)/spline(ux,v_u(i,:),u);
if mintemp > maxtemp
    temp = maxtemp;
    maxtemp = mintemp;
    mintemp = temp;
end
Lxyz(i) = mintemp;
Uxyz(i) = maxtemp;
end
L = max(Lxyz);
U = min(Uxyz);
end