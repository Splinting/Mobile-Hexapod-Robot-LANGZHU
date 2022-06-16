%worldCo2jointCo  坐标齐次变换：机身世界坐标系到腿的关节坐标系
%
%px=worldCo2jointCo(i,p0)
%px=[x;y;z]是某一条腿的关节坐标系的坐标
%i是腿号；i=1-6，此处腿编号的机械布局比较阴间
%p0=[x;y;z]是世界坐标系的坐标


function [k]=worldCo2jointCo(i,p0)
arguments 
    i (1,1)
    p0 (3,1)
end
p00=[p0;1];

global xx1 yy1 yy2 theta1 theta2 theta3

    %% 
    
    switch i
        case 1
            Tx_0=trotz(theta1,'deg')*transl(-xx1,yy1,0);
        case 2
            Tx_0=trotz(-theta2,'deg')*transl(0,-yy2,0);
        case 3
            Tx_0=trotz(theta3,'deg')*transl(xx1,yy1,0);
        case 4
            Tx_0=trotz(-theta1,'deg')*transl(-xx1,-yy1,0);
        case 5
            Tx_0=trotz(theta2,'deg')*transl(0,yy2,0);
        case 6
            Tx_0=trotz(-theta3,'deg')*transl(xx1,-yy1,0);
        otherwise
            Tx_0=zeros(4);
            disp("jijiji");
    end

    px=Tx_0*p00;
    k=px(1:3,1);
end