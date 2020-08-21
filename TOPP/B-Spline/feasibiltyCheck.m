function [L,U] = feasibiltyCheck(s,sdot,v_u,a_u,capibity)
V=capibity(1);

A = capibity(2);
Lxyz = zeros(3,1);
Uxyz = zeros(3,1);
ux = linspace(0,1,size(a_u,2));
for i = 1:3
mintemp = (A-spline(ux,a_u(i,:),s)*sdot.^2)/spline(ux,v_u(i,:),s);
maxtemp = (-A-spline(ux,a_u(i,:),s)*sdot.^2)/spline(ux,v_u(i,:),s);
if mintemp > maxtemp
    temp = maxtemp;
    maxtemp = mintemp;
    mintemp = temp;
end
Lxyz(i) = mintemp;
Uxyz(i) = maxtemp;
end


% for i = 4:6
% mintemp = -V/spline(ux,v_u(i-3,:),s);
% maxtemp = V/spline(ux,v_u(i-3,:),s);
% if mintemp > maxtemp
%     temp = maxtemp;
%     maxtemp = mintemp;
%     mintemp = temp;
% end
% Lxyz(i) = mintemp;
% Uxyz(i) = maxtemp;
% 
% end

L = max(Lxyz);
U = min(Uxyz);