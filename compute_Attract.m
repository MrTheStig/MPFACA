function Fat=compute_Attract(pos,gpos,k)% �������Ϊ��ǰ���꣬Ŀ�����꣬���泣��
%��������
% R=(pos(1)-gpos(1,1))^2+(pos(2)-gpos(1,2))^2;% ·�����Ŀ��ľ���ƽ��
% r=sqrt(R);% ·�����Ŀ��ľ���
% angles=compute_angle(pos,gpos,1);%����Ƕ�
% Fatx=k*r*cos(angles(1));
% Faty=k*r*sin(angles(1));
Fatx = (gpos(1)-pos(1))*k;
Faty = (gpos(2)-pos(2))*k;
Fat = [Fatx,Faty];%�������x�����y�������
end