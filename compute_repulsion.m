function Frep=compute_repulsion(pos,opos,goal_pos,ovel,m,n,Po,r,a,vbuff)
Rat=(pos(1)-goal_pos(1))^2+(pos(2)-goal_pos(2))^2;
rat=sqrt(Rat);
Rrei=ones(n);
rre=ones(n);
frep=zeros(n,2);
angles=compute_angle(pos,opos,n);
for i=1:n
    Rrei(i)=(pos(1)-opos(i,1))^2+(pos(2)-opos(i,2))^2;
    rre(i)=sqrt(Rrei(i))-r;
    R0=(goal_pos(1)-opos(i,1))^2+(goal_pos(2)-opos(i,2))^2;
    r0=sqrt(R0)-r;
    
%     if rre(i)>Po% 
%         Yrerx(i)=0;
%         Yrery(i)=0;
%         Yatax(i)=0;
%         Yatay(i)=0;
%     else
    if rre(i) <= Po &&( pos(1)~=opos(i,1) || pos(2)~=opos(i,2))
        frep1=m*(1/rre(i)-1/Po)*((1/rre(i))^2)*(rat^(a*2));
        frep2=a*m*((1/rre(i)-1/Po)^2)*(rat^(2*a -1));
        frep(i,:) = [(frep1+frep2)*cos(angles(i)),(frep1+frep2)*sin(angles(i))];
    frep(i,:) = frep(i,:) + (ovel(i,:)*vbuff);
    end

end
Frep = [sum(frep(:,1)),sum(frep(:,2))];
end
