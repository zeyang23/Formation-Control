% BaseFunction.m文件
function Nik_u = BaseFunction(i, k , u, NodeVector)
% 计算基函数Ni,k(u),NodeVector为节点向量

if k == 0       % 0次B样条
    if (u >= NodeVector(i+1)) && (u < NodeVector(i+2)) || ((u >= NodeVector(i+1)) && (u <= NodeVector(i+2)) && u == 1)
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % 支撑区间的长度
    if Length1 == 0.0       % 规定0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end