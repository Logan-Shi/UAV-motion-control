function [Q,Q2,Q3,v_u,a_u] = BSplineDrv(P,n,k,t,NodeVector)
NodeVector4v = NodeVector(2:end-1);
% ���ߵ�ʸ����
for i = 0:n-1
    Q(:,i+1) = k/(NodeVector4v(i+k+1)-NodeVector4v(i+1))*(P(:,i+2)-P(:,i+1));
end
[v_u,NodeVector4a] = BSpline(Q,k-1,t);

for i = 0:n-2
    Q2(:,i+1) = (k-1)/(NodeVector4a(i+k)-NodeVector4a(i+1))*(Q(:,i+2)-Q(:,i+1));
end
[a_u,NodeVector4j] = BSpline(Q2,k-2,t);

for i = 0:n-3
    Q3(:,i+1) = (k-2)/(NodeVector4j(i+k-1)-NodeVector4j(i+1))*(Q2(:,i+2)-Q2(:,i+1));
end

%     % graphing
%     subplot(1,2,1)
%     hold on
% %     plot3(Q(1, :), Q(2, :),Q(3, :),...
% %         'o','LineWidth',1,...
% %         'MarkerEdgeColor','k',...
% %         'MarkerFaceColor','g',...
% %         'MarkerSize',6);
% %     line(Q(1, :), Q(2, :), Q(3, :));
%     plot3(v_u(1,:), v_u(2,:), v_u(3,:), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
%     quiver3(v_u(1,:), v_u(2,:), v_u(3,:),a_u(1,:), a_u(2,:), a_u(3,:), 3)
% %     plot3(P(1, :), P(2, :),P(3, :),...
% %         'o','LineWidth',1,...
% %         'MarkerEdgeColor','k',...
% %         'MarkerFaceColor','r',...
% %         'MarkerSize',6);
%     grid on;axis equal
%     legend('control point','control polygon','p','v')
%     title('B-Spline Demo')
%     xlabel('x');ylabel('y');zlabel('z')
end