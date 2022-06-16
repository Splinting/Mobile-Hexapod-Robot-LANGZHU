global xx1 yy1 yy2 theta1 theta2 theta3

%% 生成六个脚
xo=[xx1,0,-xx1,xx1,0,-xx1];
yo=[-yy1,yy2,-yy1,yy1,-yy2,yy1];
zo=0;

ro=[-theta1,theta2,-theta3,theta1,-theta2,theta3]./180.*pi;

legs=cell(1,6);
for i=1:6
    legs{i}=genLeg(ro(i),xo(i),yo(i),zo,mat2str(i));
end

qz=[0,0,0];
figure(1)
view(3)
for i=1:6
    legs{i}.plot(qz,'tilesize',100);
    hold on;
end

%% 六个脚跑轨迹
%工具箱好像只能一个个跑，没啥作用……图一乐
traj=cell(1,6);
for i=1:6
traj{i}=zeros(nn,3);
traj{i}(:,1:3)=(trace_jointAng{1}./180.*pi)';

legs{i}.plot(traj{i},'trail','b.','tilesize',100);
end

%% 生成工作空间
j3bias=10.91;
% stdLeg=genLeg(0,0,0,0,'workspace');


%高机位
stdLeg.links(1).qlim=[-30,30]./180.*pi;     %三个关节角的限制
stdLeg.links(2).qlim=[-180,0]./180.*pi;
stdLeg.links(3).qlim=[90-j3bias,180-j3bias]./180.*pi;


num=1000;   %随机生成点的数量
P=zeros(num,3);

for i=1:num
    q1=stdLeg.links(1).qlim(1)+rand*(stdLeg.links(1).qlim(2)-stdLeg.links(1).qlim(1));
    q2=stdLeg.links(2).qlim(1)+rand*(stdLeg.links(2).qlim(2)-stdLeg.links(2).qlim(1));
    q3=stdLeg.links(3).qlim(1)+rand*(stdLeg.links(3).qlim(2)-stdLeg.links(3).qlim(1));

    q=[q1,q2,q3,0];
    T=stdLeg.fkine(q);
    P(i,:)=transl(T);
end

figure(2)
view(3)
plot3(P(:,1),P(:,2),P(:,3),'r.');
stdLeg.plot([0,0,0,0],'tilesize',100);
hold on

%低机位
stdLeg.links(1).qlim=[-30,30]./180.*pi;     %三个关节角的限制
stdLeg.links(2).qlim=[-180,0]./180.*pi;
stdLeg.links(3).qlim=[0,90-j3bias]./180.*pi;

P=zeros(num,3);

for i=1:num
    q1=stdLeg.links(1).qlim(1)+rand*(stdLeg.links(1).qlim(2)-stdLeg.links(1).qlim(1));
    q2=stdLeg.links(2).qlim(1)+rand*(stdLeg.links(2).qlim(2)-stdLeg.links(2).qlim(1));
    q3=stdLeg.links(3).qlim(1)+rand*(stdLeg.links(3).qlim(2)-stdLeg.links(3).qlim(1));

    q=[q1,q2,q3,0];
    T=stdLeg.fkine(q);
    P(i,:)=transl(T);
end

plot3(P(:,1),P(:,2),P(:,3),'b.');

%% 复位所有腿
% orad=origin_angle./180.*pi;

for i=1:6
    legs{i}.plot([orad(:,i)',1]);
end


