function angles=compute_angle(pos,poses,n)%angles ��������������x��ĽǶ�����,pos ��������꣬poses ��Ŀ����ϰ�����������,ȡǰn�н��м���
deltaX = ones(n);
deltaY = ones(n);
r = ones(n);
angles = ones(n);
for i=1:n%n �Ǽ�����Ŀ
    deltaX(i)=poses(i,1)-pos(1);
    deltaY(i)=poses(i,2)-pos(2);
    r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
    if deltaY(i)>=0
        theta=acos(deltaX(i)/r(i));
    else
        theta=-1*acos(deltaX(i)/r(i));
    end
    angles(i)=theta;% ����ÿ���Ƕ���Y �������棬��һ��Ԫ������Ŀ��ĽǶȣ����涼�����ϰ��ĽǶ�
end
end