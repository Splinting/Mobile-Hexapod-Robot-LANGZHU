%jointCo2worldCo  坐标齐次变换：腿的关节坐标系到机身世界坐标系
%
%p0=jointCo2worldCo(i,px)
%p0=[x;y;z]是世界坐标系的坐标
%i是腿号；i=1-6，此处腿编号的机械布局比较阴间
%px=[x;y;z]是某一条腿的关节坐标系的坐标


function [k]=jointCo2worldCo(i,px)
arguments 
    i (1,1)
    px (3,1)
end
pxx=[px;1];

global xx1 yy1 yy2 theta1 theta2 theta3

    switch i
        case 1
            Tx_0=(transl(xx1,-yy1,0)*trotz(-theta1,'deg'));
        case 2
            Tx_0=(transl(0,yy2,0)*trotz(theta2,'deg'));
        case 3
            Tx_0=(transl(-xx1,-yy1,0)*trotz(-theta3,'deg'));
        case 4
            Tx_0=(transl(xx1,yy1,0)*trotz(theta1,'deg'));
        case 5
            Tx_0=(transl(0,-yy2,0)*trotz(-theta2,'deg'));
        case 6
            Tx_0=(transl(-xx1,yy1,0)*trotz(theta3,'deg'));
        otherwise
            Tx_0=zeros(4);
            disp("jijiji");
    end


    px=Tx_0*pxx;
    k=px(1:3,1);
end