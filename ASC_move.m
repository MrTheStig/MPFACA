
path='map.txt';
[mapsize,start_pos,goal_pos,obs_info] = load_map_info(path);

G=zeros(mapsize.');
for obs=1:size(obs_info,1)
    obx = obs_info(obs,1) + floor(obs_info(obs,3)*1);
    oby = obs_info(obs,2) + floor(obs_info(obs,4)*1);
    if obx>=1 && obx<=mapsize(1) && oby>=1 && oby<=mapsize(2)
        G(obx,oby)=1;
    end
end
D=G2D(G); %距离表
K=200;                       	   %迭代次数（指蚂蚁出动  多少波）
M=50;                        	   %蚂蚁个数
MM=size(G,1);                 	   % G 地形图为01矩阵，如果为1表示障碍物 
Tau=ones(MM*MM,MM*MM);        % Tau 初始信息素矩阵
S=1 ;                         	   %最短路径的起始点
Tau=8.*Tau; 
E=MM*MM;                        %最短路径的目的点
Alpha=1;                      	   % Alpha 表征信息素重要程度的参数
Beta=8;                       	   % Beta 表征启发式因子重要程度的参数
Rho=0.3;                      	   % Rho 信息素蒸发系数
Q=3;                               % Q 信息素增加强度系数 
minkl=inf; 
mink=0; 
minl=0; 

N=size(D,1);               %N表示问题的规模（象素个数）
a=1;                     %小方格象素的边长
Ex=(mod(E,MM)-0.5);    %终止点横坐标
if Ex==-0.5 
    Ex=MM-0.5; 
end 
Ey=floor((E-1)/MM)+0.5; %终止点纵坐标
Eta=zeros(N);             %启发式信息，取为至目标点的直线距离的倒数
%以下启发式信息矩阵
for i=1:N 
    ix=(mod(i,MM)-0.5); 
    if ix==-0.5 
        ix=MM-0.5; 
    end 
    iy=floor((i-1)/MM)+0.5;  
    if i~=E 
        Eta(i)=1/((ix-Ex)^2+(iy-Ey)^2)^0.5; 
    else 
    Eta(i)=100; 
    end 
end
ROUTES=cell(K,M);     %用细胞结构存储每一代的每一只蚂蚁的爬行路线
PL=zeros(K,M);         %用矩阵存储每一代的每一只蚂蚁的爬行路线长度
                      %启动K轮蚂蚁觅食活动，每轮派出M只蚂蚁
for k=1:K
    k
    for m=1:M 
        
        %状态初始化
        W=S;                  %当前节点初始化为起始点
        Path=S;                %爬行路线初始化
        PLkm=0;               %爬行路线长度初始化
        TABUkm=ones(N);       %禁忌表初始化
        TABUkm(S)=0;          %已经在初始点了，因此要排除
        DD=D;                 %邻接矩阵初始化
        %下一步可以前往的节点
        DW=DD(W,:); 
        DW1=find(DW); 
        for j=1:length(DW1) 
           if TABUkm(DW1(j))==0 
              DW(DW1(j))=0; 
          end 
        end 
        LJD=find(DW); 
        Len_LJD=length(LJD);%可选节点的个数
        %蚂蚁未遇到食物或者陷入死胡同或者觅食停止
        t=1;%时间
        while W~=E&&Len_LJD>=1 
            %转轮赌法选择下一步怎么走
            PP=zeros(Len_LJD,1); 
            for i=1:Len_LJD
                Tau(W,LJD(i));
                PP(i)=(Tau(W,LJD(i))^Alpha)*((Eta(LJD(i)))^Beta);
            end 
            PP;
            sumpp=sum(PP); 
            PP=PP/sumpp;%建立概率分布
            Pcum(1)=PP(1); 
            Len_LJD;
            for i=2:Len_LJD 
                Pcum(i)=Pcum(i-1)+PP(i); 
            end 
            Pcum;
            Select=find(Pcum>=rand);
            to_visit=LJD(Select(1)); 
            %状态更新和记录
            Path=[Path,to_visit];       		 %路径增加
            PLkm=PLkm+DD(W,to_visit);    %路径长度增加
            W=to_visit;                   %蚂蚁移到下一个节点
            for kk=1:N 
                if TABUkm(kk)==0 
                    DD(W,kk)=0; 
                    DD(kk,W)=0; 
                end 
            end 
            TABUkm(W)=0;				%已访问过的节点从禁忌表中删除
            t = t+1;
            %更新地图
            G=zeros(mapsize.');
            for obs=1:size(obs_info,1)
                obx = obs_info(obs,1) + floor(obs_info(obs,3)*t);
                oby = obs_info(obs,2) + floor(obs_info(obs,4)*t);
                if obx>=1 && obx<=mapsize(1) && oby>=1 && oby<=mapsize(2)
                    G(obx,oby)=1;
                end
            end
            DD=G2D(G); %距离表
            DW=DD(W,:); 
            DW1=find(DW); 
            for j=1:length(DW1) 
                if TABUkm(DW1(j))==0 
                    DW(j)=0; 
                end 
            end 
            LJD=find(DW); 
            Len_LJD=length(LJD);%可选节点的个数

        end
        %记下每一代每一只蚂蚁的觅食路线和路线长度
        ROUTES{k,m}=Path; 
        if Path(end)==E 
            PL(k,m)=PLkm; 
            if PLkm<minkl 
                shortest_path=Path;
                mink=k;minl=m;minkl=PLkm; 
            end 
        else 
            PL(k,m)=0; 
        end
    end
    %更新信息素
    Delta_Tau=zeros(N,N);%更新量初始化
    for m=1:M 
        if PL(k,m)  
            ROUT=ROUTES{k,m}; 
            TS=length(ROUT)-1;%跳数
            PL_km=PL(k,m); 
            for s=1:TS 
                x=ROUT(s); 
                y=ROUT(s+1); 
                Delta_Tau(x,y)=Delta_Tau(x,y)+Q/PL_km; 
                Delta_Tau(y,x)=Delta_Tau(y,x)+Q/PL_km; 
            end 
        end 
    end 
    Tau=(1-Rho).*Tau+Delta_Tau;%信息素挥发一部分，新增加一部分
    %disp("1")
    Tau=tau_trans(mapsize,goal_pos,obs_info,Tau);%人工势场法转移信息素
end
%绘图
figure();
minPL=zeros(K,1); 
for i=1:K 
 PLK=PL(i,:); 
 Nonzero=find(PLK); 
 PLKPLK=PLK(Nonzero); 
 minPL(i)=min(PLKPLK); 
end 
 
plot(minPL); 
hold on 
grid on 
title('Trend of convergence curve'); 
xlabel('The number of iteration/Time'); 
ylabel('Minimum path length/Km');
%vis(path,shortest_path)%动态可视化


