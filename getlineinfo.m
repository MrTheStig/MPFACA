function [ data ] = getlineinfo( lineinfo )
%getlineinfo ��txt��ȡ������Ϣ���ַ���ת���ɾ���
s=regexp(lineinfo,'\s+');%���ո�ָ�
data=zeros(length(s)+1,1);
for i=1:length(s)   % ���ַ���ȫ��תΪ����, ����data��
    if i==1
        data(i)=str2num(lineinfo(1:s(i)));   
    else
        data(i)=str2num(lineinfo(s(i-1):s(i)));
    end
end
data(length(s)+1)=str2num(lineinfo(s(end):end));

end

