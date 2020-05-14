function NodeVector = U_quasi_uniform(n, k)
% ׼����B�����Ľڵ��������㣬��n+1�����ƶ��㣬k��B����
NodeVector = zeros(1, n+k+2);
piecewise = n - k + 1;       % ���ߵĶ���
if piecewise == 1       % ֻ��һ������ʱ��n = k
    for i = n+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;       % ��ֹһ������ʱ
    while flag ~= piecewise
        NodeVector(1, k+1+flag) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;
end