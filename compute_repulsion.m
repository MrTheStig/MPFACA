function Frep=compute_repulsion(pos,opos,goal_pos,ovel,m,n,Po,r,a,vbuff)
%输入参数pos为当前坐标,opos是障碍的坐标向量，goal_pos是目标坐标,m是增益常数
%计算斥力
Rat=(pos(1)-goal_pos(1))^2+(pos(2)-goal_pos(2))^2;% 路径点和目标的距离平方
rat=sqrt(Rat);% 路径点和目标的距离
Rrei=ones(n);
rre=ones(n);
frep=zeros(n,2);
angles=compute_angle(pos,opos,n);%计算角度
for i=1:n
    Rrei(i)=(pos(1)-opos(i,1))^2+(pos(2)-opos(i,2))^2;% 路径点和障碍的距离平方
    rre(i)=sqrt(Rrei(i))-r;% 路径点和障碍的距离保存在数组rrei 中
    R0=(goal_pos(1)-opos(i,1))^2+(goal_pos(2)-opos(i,2))^2;%目标和障碍距离平方
    r0=sqrt(R0)-r;
    
%     if rre(i)>Po% 如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
%         Yrerx(i)=0;
%         Yrery(i)=0;
%         Yatax(i)=0;
%         Yatay(i)=0;
%     else
    if rre(i) <= Po &&( pos(1)~=opos(i,1) || pos(2)~=opos(i,2))
        frep1=m*(1/rre(i)-1/Po)*((1/rre(i))^2)*(rat^(a*2));% 分解的Fre1 向量
        frep2=a*m*((1/rre(i)-1/Po)^2)*(rat^(2*a -1));% 分解的Fre2 向量
        frep(i,:) = [(frep1+frep2)*cos(angles(i)),(frep1+frep2)*sin(angles(i))];
    frep(i,:) = frep(i,:) + (ovel(i,:)*vbuff);
    end%判断距离是否在障碍影响范围内

end
Frep = [sum(frep(:,1)),sum(frep(:,2))];%叠加求合力
end