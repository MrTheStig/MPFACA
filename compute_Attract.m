function Fat=compute_Attract(pos,gpos,k)% 输入参数为当前坐标，目标坐标，增益常数
%计算引力
% R=(pos(1)-gpos(1,1))^2+(pos(2)-gpos(1,2))^2;% 路径点和目标的距离平方
% r=sqrt(R);% 路径点和目标的距离
% angles=compute_angle(pos,gpos,1);%计算角度
% Fatx=k*r*cos(angles(1));
% Faty=k*r*sin(angles(1));
Fatx = (gpos(1)-pos(1))*k;
Faty = (gpos(2)-pos(2))*k;
Fat = [Fatx,Faty];%输出引力x方向和y方向分量
end