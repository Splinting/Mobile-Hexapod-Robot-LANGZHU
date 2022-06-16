%% 
L=2*80;    %前进距离
H=50;       %腿抬高高度
nn=20;      %采样数（4的倍数）
%% 
T=1;
t=0:T/nn:T/2;
s=t/T;

%抄的轨迹函数
xs=L*(-16*power(s,3)+12*power(s,2)-s-1/4);
xs=-xs;     %right
zs=zeros(1,nn/2+1);
for n=1:nn/2+1
    if(t(n)<0.25*T)
        zs(n)=H*(-128*power(s(n),3)+48*power(s(n),2));
    else
        zs(n)=H*(-128*power((0.5*T-t(n)/T),3)+48*power((0.5*T-t(n)/T),2));
    end
end
xp=L*(-s+1/4);
xp=-xp;     %right
zp=zeros(1,nn/2+1);

plot(xs,zs);
hold on;
plot(xp,zp);

%% 生成步态轨迹的世界坐标

baseBias={[xxx1;-yyy1;zzz1],   [0;yyy2;zzz1],   [-xxx1;-yyy1;zzz1],...
           [xxx1;yyy1;zzz1],    [0;-yyy2;zzz1],  [-xxx1;yyy1;zzz1]};

trace_worldCo=cell(1,6);
for i=1:6
    trace_worldCo{i}=zeros(3,nn);
    if i==1||i==2||i==3           %同步态的三条腿/同时着地的三条腿

        for j=(nn/4+1):nn/2       %从s的中间开始，半个s步态
            trace_worldCo{i}(:,j-nn/4)=baseBias{i}+[0;xs(j);zs(j)];% (0; Δy; Δz)
        end
        for j=1:nn/2              %半个s后接一个p步态
            trace_worldCo{i}(:,nn/4+j)=baseBias{i}+[0;xp(j);zp(j)];
        end
        for j=1:nn/4              %再接剩下半个s  后半s--p--前半s
            trace_worldCo{i}(:,j+(3*nn/4))=baseBias{i}+[0;xs(j);zs(j)];
        end

    else    %i==4||i==5||i==6      另外三条s和p步态反过来

        for j=(nn/4+1):nn/2   
            trace_worldCo{i}(:,j-nn/4)=baseBias{i}+[0;xp(j);zp(j)];%xp和xs顺序交换
        end
        for j=1:nn/2   
            trace_worldCo{i}(:,nn/4+j)=baseBias{i}+[0;xs(j);zs(j)];
        end
        for j=1:nn/4              %后半p--s--前半p
            trace_worldCo{i}(:,j+(3*nn/4))=baseBias{i}+[0;xp(j);zp(j)];
        end
    end
end

figure(2)
for i=1:6
    plot3(trace_worldCo{i}(1,:),trace_worldCo{i}(2,:),trace_worldCo{i}(3,:));
    hold on;
end
xlabel('x');
ylabel('y');
zlabel('z');
%% 获取步态轨迹的关节坐标系坐标
trace_jointCo=cell(1,6);
for i=1:6
    for j=1:nn
        [trace_jointCo{i}(:,j)]=worldCo2jointCo(i,trace_worldCo{i}(:,j));
    end
end

%% 逆运动学 获取步态轨迹的对应关节角
trace_jointAng=cell(1,6);
for i=1:6
    trace_jointAng{i}=zeros(3,nn);
    for j=1:nn
        [trace_jointAng{i}(:,j)]=jointCo2jointAng(trace_jointCo{i}(:,j));
    end
end



