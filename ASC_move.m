
path='map.txt';
[mapsize,start_pos,goal_pos,obs_info] = load_map_info(path);

G=zeros(mapsize.');
for obs=1:size(obs_info,1)
    obx = obs_info(obs,1) + floor(obs_info(obs,3)*1);
    oby = obs_info(obs,2) + floor(obs_info(obs,4)*1);
    if obx>=1 && obx<=mapsize(1) && oby>=1 && oby<=mapsize(2)
        G(obx,oby)=1;
    end
end
D=G2D(G); %�����
K=200;                       	   %����������ָ���ϳ���  ���ٲ���
M=50;                        	   %���ϸ���
MM=size(G,1);                 	   % G ����ͼΪ01�������Ϊ1��ʾ�ϰ��� 
Tau=ones(MM*MM,MM*MM);        % Tau ��ʼ��Ϣ�ؾ���
S=1 ;                         	   %���·������ʼ��
Tau=8.*Tau; 
E=MM*MM;                        %���·����Ŀ�ĵ�
Alpha=1;                      	   % Alpha ������Ϣ����Ҫ�̶ȵĲ���
Beta=8;                       	   % Beta ��������ʽ������Ҫ�̶ȵĲ���
Rho=0.3;                      	   % Rho ��Ϣ������ϵ��
Q=3;                               % Q ��Ϣ������ǿ��ϵ�� 
minkl=inf; 
mink=0; 
minl=0; 

N=size(D,1);               %N��ʾ����Ĺ�ģ�����ظ�����
a=1;                     %С�������صı߳�
Ex=(mod(E,MM)-0.5);    %��ֹ�������
if Ex==-0.5 
    Ex=MM-0.5; 
end 
Ey=floor((E-1)/MM)+0.5; %��ֹ��������
Eta=zeros(N);             %����ʽ��Ϣ��ȡΪ��Ŀ����ֱ�߾���ĵ���
%��������ʽ��Ϣ����
for i=1:N 
    ix=(mod(i,MM)-0.5); 
    if ix==-0.5 
        ix=MM-0.5; 
    end 
    iy=floor((i-1)/MM)+0.5;  
    if i~=E 
        Eta(i)=1/((ix-Ex)^2+(iy-Ey)^2)^0.5; 
    else 
    Eta(i)=100; 
    end 
end
ROUTES=cell(K,M);     %��ϸ���ṹ�洢ÿһ����ÿһֻ���ϵ�����·��
PL=zeros(K,M);         %�þ���洢ÿһ����ÿһֻ���ϵ�����·�߳���
                      %����K��������ʳ���ÿ���ɳ�Mֻ����
for k=1:K
    k
    for m=1:M 
        
        %״̬��ʼ��
        W=S;                  %��ǰ�ڵ��ʼ��Ϊ��ʼ��
        Path=S;                %����·�߳�ʼ��
        PLkm=0;               %����·�߳��ȳ�ʼ��
        TABUkm=ones(N);       %���ɱ��ʼ��
        TABUkm(S)=0;          %�Ѿ��ڳ�ʼ���ˣ����Ҫ�ų�
        DD=D;                 %�ڽӾ����ʼ��
        %��һ������ǰ���Ľڵ�
        DW=DD(W,:); 
        DW1=find(DW); 
        for j=1:length(DW1) 
           if TABUkm(DW1(j))==0 
              DW(DW1(j))=0; 
          end 
        end 
        LJD=find(DW); 
        Len_LJD=length(LJD);%��ѡ�ڵ�ĸ���
        %����δ����ʳ�������������ͬ������ʳֹͣ
        t=1;%ʱ��
        while W~=E&&Len_LJD>=1 
            %ת�ֶķ�ѡ����һ����ô��
            PP=zeros(Len_LJD,1); 
            for i=1:Len_LJD
                Tau(W,LJD(i));
                PP(i)=(Tau(W,LJD(i))^Alpha)*((Eta(LJD(i)))^Beta);
            end 
            PP;
            sumpp=sum(PP); 
            PP=PP/sumpp;%�������ʷֲ�
            Pcum(1)=PP(1); 
            Len_LJD;
            for i=2:Len_LJD 
                Pcum(i)=Pcum(i-1)+PP(i); 
            end 
            Pcum;
            Select=find(Pcum>=rand);
            to_visit=LJD(Select(1)); 
            %״̬���ºͼ�¼
            Path=[Path,to_visit];       		 %·������
            PLkm=PLkm+DD(W,to_visit);    %·����������
            W=to_visit;                   %�����Ƶ���һ���ڵ�
            for kk=1:N 
                if TABUkm(kk)==0 
                    DD(W,kk)=0; 
                    DD(kk,W)=0; 
                end 
            end 
            TABUkm(W)=0;				%�ѷ��ʹ��Ľڵ�ӽ��ɱ���ɾ��
            t = t+1;
            %���µ�ͼ
            G=zeros(mapsize.');
            for obs=1:size(obs_info,1)
                obx = obs_info(obs,1) + floor(obs_info(obs,3)*t);
                oby = obs_info(obs,2) + floor(obs_info(obs,4)*t);
                if obx>=1 && obx<=mapsize(1) && oby>=1 && oby<=mapsize(2)
                    G(obx,oby)=1;
                end
            end
            DD=G2D(G); %�����
            DW=DD(W,:); 
            DW1=find(DW); 
            for j=1:length(DW1) 
                if TABUkm(DW1(j))==0 
                    DW(j)=0; 
                end 
            end 
            LJD=find(DW); 
            Len_LJD=length(LJD);%��ѡ�ڵ�ĸ���

        end
        %����ÿһ��ÿһֻ���ϵ���ʳ·�ߺ�·�߳���
        ROUTES{k,m}=Path; 
        if Path(end)==E 
            PL(k,m)=PLkm; 
            if PLkm<minkl 
                shortest_path=Path;
                mink=k;minl=m;minkl=PLkm; 
            end 
        else 
            PL(k,m)=0; 
        end
    end
    %������Ϣ��
    Delta_Tau=zeros(N,N);%��������ʼ��
    for m=1:M 
        if PL(k,m)  
            ROUT=ROUTES{k,m}; 
            TS=length(ROUT)-1;%����
            PL_km=PL(k,m); 
            for s=1:TS 
                x=ROUT(s); 
                y=ROUT(s+1); 
                Delta_Tau(x,y)=Delta_Tau(x,y)+Q/PL_km; 
                Delta_Tau(y,x)=Delta_Tau(y,x)+Q/PL_km; 
            end 
        end 
    end 
    Tau=(1-Rho).*Tau+Delta_Tau;%��Ϣ�ػӷ�һ���֣�������һ����
    %disp("1")
    Tau=tau_trans(mapsize,goal_pos,obs_info,Tau);%�˹��Ƴ���ת����Ϣ��
end
%��ͼ
figure();
minPL=zeros(K,1); 
for i=1:K 
 PLK=PL(i,:); 
 Nonzero=find(PLK); 
 PLKPLK=PLK(Nonzero); 
 minPL(i)=min(PLKPLK); 
end 
 
plot(minPL); 
hold on 
grid on 
title('Trend of convergence curve'); 
xlabel('The number of iteration/Time'); 
ylabel('Minimum path length/Km');
%vis(path,shortest_path)%��̬���ӻ�


