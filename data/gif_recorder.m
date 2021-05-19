clear all; close all;
addpath(genpath('..'));
static_quadrotor_plot(quad_a.position, quad_a.attitude, quad_a.L, 1);
hold on
plot3(pt(1,:), pt(2,:), pt(3,:), 'b');
plot3(quad_a.position_H(1,:), quad_a.position_H(2,:), quad_a.position_H(3,:),'r');
title('real time result');
hold off
% ax = [-1,6,-1,6,-1,3]; % for scan
% ax = [-2.5,3.5,-2,4,-1,1.5]; % for circle
ax = [-10,1,-6,1,-1,3];
view(135,45)
axis(ax);
img = frame2im(getframe(gcf));
[img, map] = rgb2ind(img, 256);
if mod(k,10) == 2
    if k == 2
        imwrite(img, map, 'test.gif', 'gif', 'Loopcount', inf, 'DelayTime', dt);
    else
        imwrite(img, map, 'test.gif', 'gif', 'WriteMode', 'append', 'DelayTime', dt);
    end
end

rmpath(genpath('..'));