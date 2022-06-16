%jointCo2jointAng 一条腿的逆运动学：关节坐标->关节角
%
%q=jointCo2jointAng(px)
%q=[r1;r2;r3]为表示关节角的列向量，顺序参照建模的关节顺序
%px=[x;y;z]为关节坐标系坐标

function [q]=jointCo2jointAng(px)%px 关节坐标系坐标
arguments 
    px (3,1)
end

global l1 l2 l3
j3bias=10.91;

    x=px(1);
    y=px(2);
    z=px(3);

    A=sqrt(x*x+y*y)-l1;
    B=-z;

    if(A>0)
        assert((A*A+(z-l2)*(z-l2))>l3*l3,"不在工作空间内!");
        assert((A*A+z*z)<(l2+l3)*(l2+l3),"不在工作空间内!");
    else
        assert((A*A+z*z)>(l3-l2)*(l3-l2),"不在工作空间内!");
        assert((A*A+(z+l2)*(z+l2))<l3*l3,"不在工作空间内!");
    end

   

    %% 
    %   r1
    r1=atan2(y,x)*(180/pi);
        assert(r1>-60,"r1<-60, dangerous!");
        assert(r1<60,"r1>60, dangerous!");

    if((A*A+z*z)>(l2*l2+l3*l3))
%         disp("高机位捏");
            %r3
            ccos=acos((l2*l2+l3*l3-A*A-B*B)/(2*l2*l3))*(180/pi);
            if(ccos>90)
                ccos=180-ccos;
            end
            r3=180-ccos-j3bias;
                assert(r3>90-j3bias,"r3<79.09, dangerous! (可能是低机位的解)");
                assert(r3<(180-j3bias),"r3>169.09, dangerous!");
            %r2
            delta=atan((l3*sin(ccos/180*pi))/(l2+(l3*cos(ccos/180*pi))));
            ttan=atan(B/A);
            r2=-90-(ttan-delta)/pi*180;
                assert(r2<0,"r2>0, dangerous!(可能是低机位的解)")
                assert(r2>-180,"r2<-180, dangerous!")

    else
%         disp("低机位捏");
            %   r2 
            ccos=acos((A*A+B*B+l2*l2-l3*l3)/(2*sqrt(A*A+B*B)*l2))*(180/pi);
            r2=ccos-(atan2(B,A)*(180/pi))-90;
                assert(r2>=-180,"r2<=-180, dangerous!");
                assert(r2<0,"r2>0, dangerous!");
            %   r3
            ccos2=acos((-A*A-B*B+l2*l2+l3*l3)/(2*l2*l3))*(180/pi);
                if ccos2>=90
                    ccos2=180-ccos2;
                end
            r3=ccos2-j3bias;
                assert(r3>0,"r3<0, dangerous!");
                assert(r3<(90-j3bias),"r3>79.09, dangerous!（可能是高机位的解）");
                
    end

    
    q=[r1;r2;r3];
end


 