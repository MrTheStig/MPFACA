function [] = draw_map( map )
%���Ƶ�ͼ
%figure(); 
MM=size(map,1);                 	   % G ����ͼΪ01�������Ϊ1��ʾ�ϰ��� 

for i=1:MM 
    for j=1:MM
        x1=i-1;y1=j-1; 
        x2=i;y2=j-1; 
        x3=i;y3=j; 
        x4=i-1;y4=j; 
        if map(i,j)==1 
            fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]); 
        else  
            fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]); 
        end
        hold on; 
    end 
end
axis([0,MM,0,MM]) 
end

