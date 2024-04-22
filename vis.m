function [ ] = vis(map_path,uav_path)

%vis 可视化结果
[mapsize,~,~,obs_info] = load_map_info(map_path);
MM=mapsize(1);
time=100;
M = moviein(time);
Px=zeros(size(uav_path,2),1);
Py=zeros(size(uav_path,2),1);

for t=1:time
    %更新路径
    if t>1 && t<=1+size(uav_path,2)
        poid=uav_path(t-1);
        ix=(mod(poid,MM)-0.5); 
        if ix==-0.5 
            ix=MM-0.5; 
        end 
        iy=floor((poid-1)/MM)+0.5;
        Px(t-1)=ix;
        Py(t-1)=iy;
    end
    G=zeros(mapsize.');
    for obs=1:size(obs_info,1)
        obx = obs_info(obs,1) + floor(obs_info(obs,3)*(t-1));
        oby = obs_info(obs,2) + floor(obs_info(obs,4)*(t-1));
        if obx>=1 && obx<=mapsize(1) && oby>=1 && oby<=mapsize(2)
            G(obx,oby)=1;
        end
    end
    draw_map(G);
    if t>2
        if t<=1+size(uav_path,2)
            sp = t-1;
        else
            sp = size(uav_path,2);
        end
        plot(Px(1:sp),Py(1:sp))
    end
    M(t) = getframe;
    clf;
end
movie(M);

end

