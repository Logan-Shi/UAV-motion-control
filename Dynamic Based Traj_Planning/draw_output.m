
figure

subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('x');

subplot(2,2,2)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('y');

subplot(2,2,3)
plot(out.STATES(:,1), out.STATES(:,6), 'r')
title('z');

subplot(2,2,4)
plot(out.STATES(:,1), out.STATES(:,12), 'r')
title('psi');

figure
plot3(out.STATES(:,2),out.STATES(:,4),out.STATES(:,6))
title('trajectory')

figure
plot(out.STATES(:,1),out.STATES(:,14))
hold on
plot(out.STATES(:,1),out.STATES(:,15))
plot(out.STATES(:,1),out.STATES(:,16))
plot(out.STATES(:,1),out.STATES(:,17))
legend('omega1','omega2','omega3','omega4')