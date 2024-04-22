function [ output_Tau ] = tau_trans( mapsize,goal_pos,obs_info,input_Tau )
%tau_trans 人工势场法转移信息素
output_Tau = zeros(size(input_Tau));
[tau_ex,tau_ey] = find(input_Tau>0.001);%筛选掉信息素过小的
MM = mapsize(1);

for i=1:size(tau_ex,1)
    tau_v = input_Tau(tau_ex(i),tau_ey(i));%信息素值
    taid = tau_ex(i);
    ix=(mod(taid,MM));
    if ix==0
        ix=MM;
    end
    iy=floor((taid-1)/MM)+1;
    pos = [ix,iy];%信息素位置
    opos=obs_info(:,1:2);%障碍物位置
    ovel=obs_info(:,3:4);%障碍物速度
    %调用计算引力模块
    Fat = compute_Attract(pos,goal_pos,1000);% 计算出目标对物体的引力在x,y 方向的两个分量值。
    %调用计算斥力模块
    Frep=compute_repulsion(pos,opos,goal_pos,ovel,100,size(obs_info,1),2,0,1,0.0);
    Fsumx=Fat(1)-Frep(1);%x方向的合力
    Fsumy=Fat(2)-Frep(2);%y方向的合力
    move_angle=atan(Fsumy/Fsumx);% 合力与x 轴方向的夹角向量
    %转移方向
    dx=0;
    dy=0;
    if abs(move_angle) <= pi*3/8
        if sign(Fsumx)>0
            dx = 1;
        else
            dx = -1;
        end
    end
    if abs(move_angle) > pi/8
        if sign(Fsumy)>0
            dy = 1;
        else
            dy = -1;
        end
    end
    tran_a=0.1;%转移系数
    %斜向移动
    if abs(dx)+abs(dy) == 2
        if ix+dx>=1 && ix+dx<=mapsize(1) && iy+dy>=1 && iy+dy<=mapsize(2)
            tran_v = 0.33*tran_a*tau_v;%转移的信息素量
            re_v = (1)*tau_v;%保留的信息素量
        else
            tran_v = 0;%转移的信息素量
            re_v = tau_v;%保留的信息素量
        end
    elseif abs(dx)+abs(dy)==1
        if ix+dx>=1 && ix+dx<=mapsize(1) && iy+dy>=1 && iy+dy<=mapsize(2)
            tran_v = 0.53*tran_a*tau_v;%转移的信息素量
            re_v = (1)*tau_v;%保留的信息素量
        else
            tran_v = 0;%转移的信息素量
            re_v = tau_v;%保留的信息素量
        end
    else
        tran_v = 0;%转移的信息素量
        re_v = tau_v;%保留的信息素量
    end
    if tran_v~=0
        tx = (iy+dy-1)*MM+ix+dx;
        output_Tau(tx,tau_ey(i)) = tran_v;
    end
    tx = (iy-1)*MM+ix;
    tau_ex(i);
    output_Tau(tx,tau_ey(i)) = re_v;
    
end
end

