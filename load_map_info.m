function [mapsize,start_pos,goal_pos,obs_info] = load_map_info(path)
%load_map_info 从txt读取地图信息
mapinfo = fopen(path);
lineinfo = fgetl(mapinfo);
lineid =0;
while ischar(lineinfo)
    if lineid == 0
        %读取地图大小
        mapsize = getlineinfo(lineinfo);
    elseif lineid == 1
        %读取起点终点
        data = getlineinfo(lineinfo);
        start_pos = data(1:2);
        goal_pos = data(3:4);
    elseif lineid == 2
        %读取障碍物个数
        %data = getlineinfo(lineinfo);
        obs_num = str2num(lineinfo);
        %初始化障碍物
        obs_info = zeros(obs_num,4);
    elseif lineid > 2 && lineid <= (2+obs_num)
        %读取障碍物信息
        data = getlineinfo(lineinfo);
        obs_info(lineid-2,:)=data;
    end
    lineinfo = fgetl(mapinfo);%读取下一行
    lineid = lineid + 1;
end
end

