function a = rotor2acc(rotorspeeds, attitude, rdot, m, g, k, kd)
    % author: LZY
    % last edited: ZRT
    % change to the file:
    % 原文件名 acceleration, 为防止歧义, 改名为 rotor2acc
    % 直接输入旋转矩阵attitude，不再使用欧拉角
    
    %   rotorspeeds 四个电机的转速
    %   attitude 位姿 变换矩阵
    %   rdot 线速度矢量 粘性阻尼 阻力与线速度成正比
    gravity = [0; 0; -g];
    R = (attitude);
    T = R * thrust(rotorspeeds, k);
    Fd = -kd * rdot;
    a = gravity + 1 / m * T + Fd;
end
