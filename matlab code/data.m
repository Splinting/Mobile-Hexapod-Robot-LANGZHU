clear
clc
%机械位形信息
    %三条腿长
    global l1 l2 l3
    l1=45;
    l2=93.5;
    l3=124.14;
    %六个关节base到中心（世界坐标系原点）的距离/角度信息
    global xx1 yy1 yy2 theta1 theta2 theta3
    xx1=115;
    yy1=65;
    yy2=95;
    theta1=80;
    theta2=90;
    theta3=100;
    %六个脚在地面的接触点到中心的初始距离信息（初始偏置）
    global yyy1 yyy2 xxx1 zzz1
    %低机位
%     xxx1=136.27;
%     yyy1=191.37;
%     yyy2=233.14;
%     zzz1=-67.38;

    %高机位
    xxx1=142.15;
    yyy1=224.71;
    yyy2=257;
    zzz1=-118;

%     %% 
% basexx=[xx1,0,-xx1,-xx1,0,xx1];
% baseyy=[-yy1,-yy2,-yy1,yy1,yy2,yy1];
% basezz=zeros(1,6);
% 
% fill(basexx,baseyy,'b');
% hold on
% 
% base={[xx1;-yy1;0],   [0;yy2;0],   [-xx1;-yy1;0],...
%        [xx1;yy1;0],    [0;-yy2;0],  [-xx1;yy1;0]};
% for i=1:6
% plot3(base{i}(1),base{i}(2),base{i}(3),'o');
% hold on
% end
