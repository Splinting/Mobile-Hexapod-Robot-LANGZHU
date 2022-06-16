%% 
Ang=45*pi/180;     %一次的旋转量
H=30;              %腿抬高高度
nn=20;             %采样数（4的倍数）
%% 
LL1=sqrt(xxx1*xxx1+yyy1*yyy1);
LL2=yyy2;       %sqrt(0*0+yyy2*yyy2)

L1=sqrt(2*LL1*LL1*(1-cos(Ang)));     %4条腿的移动距离
L2=sqrt(2*LL2*LL2*(1-cos(Ang)));     %中间2条腿的移动距离

T=1;
t=0:T/nn:T/2;
s=t/T;

%抄的轨迹函数
xs1=L1*(-16*power(s,3)+12*power(s,2)-s-1/4);
xs1=-xs1;       %clkwise
zs1=zeros(1,nn/2+1);
for n=1:nn/2+1
    if(t(n)<0.25*T)
        zs1(n)=H*(-128*power(s(n),3)+48*power(s(n),2));
    else
        zs1(n)=H*(-128*power((0.5*T-t(n)/T),3)+48*power((0.5*T-t(n)/T),2));
    end
end
xp1=L1*(-s+1/4);
xp1=-xp1;       %clkwise
zp1=zeros(1,nn/2+1);

xs2=L2*(-16*power(s,3)+12*power(s,2)-s-1/4);
xs2=-xs2;       %clkwise
zs2=zeros(1,nn/2+1);
for n=1:nn/2+1
    if(t(n)<0.25*T)
        zs2(n)=H*(-128*power(s(n),3)+48*power(s(n),2));
    else
        zs2(n)=H*(-128*power((0.5*T-t(n)/T),3)+48*power((0.5*T-t(n)/T),2));
    end
end
xp2=L2*(-s+1/4);
xp2=-xp2;       %clkwise
zp2=zeros(1,nn/2+1);
% 
% plot(xs1,zs1);
% hold on;
% plot(xp1,zp1);
% hold on;
% plot(xs2,zs2);
% hold on;
% plot(xp2,zp2);

%% 生成步态轨迹的世界坐标

baseBias={[xxx1;-yyy1;zzz1],   [0;yyy2;zzz1],   [-xxx1;-yyy1;zzz1],...
           [xxx1;yyy1;zzz1],    [0;-yyy2;zzz1],  [-xxx1;yyy1;zzz1]};

global theta1 theta2 theta3

theta=[theta1,-theta2,theta3,-theta1,theta2,-theta3].*pi./180;

trace_worldCo=cell(1,6);
for i=1:6
    trace_worldCo{i}=zeros(3,nn);
    switch i
        case {1,3}         
    
            for j=(nn/4+1):nn/2       %从s的中间开始，半个s步态
                trace_worldCo{i}(:,j-nn/4)=baseBias{i}+[xs1(j)*sin(theta(i));xs1(j)*cos(theta(i));zs1(j)];% (Δx*sin(θ); Δy*cos(θ); Δz)
            end
            for j=1:nn/2              %半个s后接一个p步态
                trace_worldCo{i}(:,nn/4+j)=baseBias{i}+[xp1(j)*sin(theta(i));xp1(j)*cos(theta(i));zp1(j)];
            end
            for j=1:nn/4              %再接剩下半个s  后半s--p--前半s
                trace_worldCo{i}(:,j+(3*nn/4))=baseBias{i}+[xs1(j)*sin(theta(i));xs1(j)*cos(theta(i));zs1(j)];
            end
        case 2
            for j=(nn/4+1):nn/2       %从s的中间开始，半个s步态
                trace_worldCo{i}(:,j-nn/4)=baseBias{i}+[xs2(j)*sin(theta(i));xs2(j)*cos(theta(i));zs2(j)];% (Δx*sin(θ); Δy*cos(θ); Δz)
            end
            for j=1:nn/2              %半个s后接一个p步态
                trace_worldCo{i}(:,nn/4+j)=baseBias{i}+[xp2(j)*sin(theta(i));xp2(j)*cos(theta(i));zp2(j)];
            end
            for j=1:nn/4              %再接剩下半个s  后半s--p--前半s
                trace_worldCo{i}(:,j+(3*nn/4))=baseBias{i}+[xs2(j)*sin(theta(i));xs2(j)*cos(theta(i));zs2(j)];
            end
            
        case {4,6}
            for j=(nn/4+1):nn/2   
                trace_worldCo{i}(:,j-nn/4)=baseBias{i}+[xp1(j)*sin(theta(i));xp1(j)*cos(theta(i));zp1(j)];%xp和xs顺序交换
            end
            for j=1:nn/2   
                trace_worldCo{i}(:,nn/4+j)=baseBias{i}+[xs1(j)*sin(theta(i));xs1(j)*cos(theta(i));zs1(j)];
            end
            for j=1:nn/4              %后半p--s--前半p
                trace_worldCo{i}(:,j+(3*nn/4))=baseBias{i}+[xp1(j)*sin(theta(i));xp1(j)*cos(theta(i));zp1(j)];
            end
        otherwise   %case 5
            for j=(nn/4+1):nn/2   
                trace_worldCo{i}(:,j-nn/4)=baseBias{i}+[xp2(j)*sin(theta(i));xp2(j)*cos(theta(i));zp2(j)];%xp和xs顺序交换
            end
            for j=1:nn/2   
                trace_worldCo{i}(:,nn/4+j)=baseBias{i}+[xs2(j)*sin(theta(i));xs2(j)*cos(theta(i));zs2(j)];
            end
            for j=1:nn/4              %后半p--s--前半p
                trace_worldCo{i}(:,j+(3*nn/4))=baseBias{i}+[xp2(j)*sin(theta(i));xp2(j)*cos(theta(i));zp2(j)];
            end
    end
end

% figure(2)
% for i=1:6
%     plot3(trace_worldCo{i}(1,:),trace_worldCo{i}(2,:),trace_worldCo{i}(3,:));
%     hold on;
% end
% xlabel('x');
% ylabel('y');
% zlabel('z');
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



