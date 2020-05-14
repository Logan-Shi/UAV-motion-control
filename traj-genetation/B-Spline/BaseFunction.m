% BaseFunction.m�ļ�
function Nik_u = BaseFunction(i, k , u, NodeVector)
% ���������Ni,k(u),NodeVectorΪ�ڵ�����

if k == 0       % 0��B����
    if (u >= NodeVector(i+1)) && (u < NodeVector(i+2))
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % ֧������ĳ���
    if Length1 == 0.0       % �涨0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end