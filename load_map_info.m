function [mapsize,start_pos,goal_pos,obs_info] = load_map_info(path)
%load_map_info 
mapinfo = fopen(path);
lineinfo = fgetl(mapinfo);
lineid =0;
while ischar(lineinfo)
    if lineid == 0
        mapsize = getlineinfo(lineinfo);
    elseif lineid == 1
        data = getlineinfo(lineinfo);
        start_pos = data(1:2);
        goal_pos = data(3:4);
    elseif lineid == 2
        %data = getlineinfo(lineinfo);
        obs_num = str2num(lineinfo);
        obs_info = zeros(obs_num,4);
    elseif lineid > 2 && lineid <= (2+obs_num)
        data = getlineinfo(lineinfo);
        obs_info(lineid-2,:)=data;
    end
    lineinfo = fgetl(mapinfo);
    lineid = lineid + 1;
end
end
