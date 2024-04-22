function [ output_Tau ] = tau_trans( mapsize,goal_pos,obs_info,input_Tau )
%tau_trans 
output_Tau = zeros(size(input_Tau));
[tau_ex,tau_ey] = find(input_Tau>0.001);
MM = mapsize(1);

for i=1:size(tau_ex,1)
    tau_v = input_Tau(tau_ex(i),tau_ey(i));
    taid = tau_ex(i);
    ix=(mod(taid,MM));
    if ix==0
        ix=MM;
    end
    iy=floor((taid-1)/MM)+1;
    pos = [ix,iy];
    opos=obs_info(:,1:2);
    ovel=obs_info(:,3:4);
    Fat = compute_Attract(pos,goal_pos,1000);
    Frep=compute_repulsion(pos,opos,goal_pos,ovel,100,size(obs_info,1),2,0,1,0.0);
    Fsumx=Fat(1)-Frep(1);
    Fsumy=Fat(2)-Frep(2);
    move_angle=atan(Fsumy/Fsumx);
    dx=0;
    dy=0;
    if abs(move_angle) <= pi*3/8
        if sign(Fsumx)>0
            dx = 1;
        else
            dx = -1;
        end
    end
    if abs(move_angle) > pi/8
        if sign(Fsumy)>0
            dy = 1;
        else
            dy = -1;
        end
    end
    tran_a=0.1;
    if abs(dx)+abs(dy) == 2
        if ix+dx>=1 && ix+dx<=mapsize(1) && iy+dy>=1 && iy+dy<=mapsize(2)
            tran_v = 0.33*tran_a*tau_v;
            re_v = (1)*tau_v;
        else
            tran_v = 0;
            re_v = tau_v;
        end
    elseif abs(dx)+abs(dy)==1
        if ix+dx>=1 && ix+dx<=mapsize(1) && iy+dy>=1 && iy+dy<=mapsize(2)
            tran_v = 0.53*tran_a*tau_v;
            re_v = (1)*tau_v;
        else
            tran_v = 0;
            re_v = tau_v;
        end
    else
        tran_v = 0;
        re_v = tau_v;
    end
    if tran_v~=0
        tx = (iy+dy-1)*MM+ix+dx;
        output_Tau(tx,tau_ey(i)) = tran_v;
    end
    tx = (iy-1)*MM+ix;
    tau_ex(i);
    output_Tau(tx,tau_ey(i)) = re_v;
    
end
end
