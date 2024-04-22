function [ data ] = getlineinfo( lineinfo )
%getlineinfo 将txt读取的行信息从字符串转换成矩阵
s=regexp(lineinfo,'\s+');%按空格分隔
data=zeros(length(s)+1,1);
for i=1:length(s)   % 将字符串全部转为数组, 存于data中
    if i==1
        data(i)=str2num(lineinfo(1:s(i)));   
    else
        data(i)=str2num(lineinfo(s(i-1):s(i)));
    end
end
data(length(s)+1)=str2num(lineinfo(s(end):end));

end

