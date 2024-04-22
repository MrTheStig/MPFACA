function angles=compute_angle(pos,poses,n)%angles 是引力，斥力与x轴的角度向量,pos 是起点坐标，poses 是目标或障碍的坐标向量,取前n行进行计算
deltaX = ones(n);
deltaY = ones(n);
r = ones(n);
angles = ones(n);
for i=1:n%n 是计算数目
    deltaX(i)=poses(i,1)-pos(1);
    deltaY(i)=poses(i,2)-pos(2);
    r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
    if deltaY(i)>=0
        theta=acos(deltaX(i)/r(i));
    else
        theta=-1*acos(deltaX(i)/r(i));
    end
    angles(i)=theta;% 保存每个角度在Y 向量里面，第一个元素是与目标的角度，后面都是与障碍的角度
end
end