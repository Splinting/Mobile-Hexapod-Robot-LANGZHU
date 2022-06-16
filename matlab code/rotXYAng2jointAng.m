%rotXYAng2jointAng 由机身要转动的roll和pitch角算出六个脚的关节角
%
%[k]=rotXYAng2jointAng(rx,ry)
%k=3*6的矩阵，包含六个脚的所有角度，可以用k(i)索引（i=1-18）；六列对应1-6号脚
%k=[r1;r4;...;r16
%   r2;r5;...;r17
%   r3;r6;...;r18]
% rx,     ry 是角度制（degree）的旋转角
% roll   pitch

function [k]=rotXYAng2jointAng(rx,ry)
k=zeros(3,6);

global yyy1 yyy2 xxx1 zzz1 
%机身脚位置初始偏置
xo=[xxx1,0,-xxx1,xxx1,0,-xxx1];
yo=[-yyy1,yyy2,-yyy1,yyy1,-yyy2,yyy1];
zo=[zzz1,zzz1,zzz1,zzz1,zzz1,zzz1];


%先由rx，ry算到x0y0z0（世界坐标系的目标位置）
p0=zeros(3,6);
p0(1,:)=xo.*cos(ry*(pi/180));
p0(2,:)=yo.*cos(rx*(pi/180));
p0(3,:)=zo-xo.*sin(ry*(pi/180))+yo.*sin(rx*(pi/180));

% plot3(p0(1,:),p0(2,:),p0(3,:));
% xlabel('x');
% ylabel('y');
% zlabel('z');
% hold on

        function qn=findNext(ii,p00)
        cnt=0;
        pn=p00;
        flag=0;
        while(cnt<=3)
            pn(3)=pn(3)-zzz1;
                if(p00(3)>-117.9)
%                     disp("低机位捏");
                    pn=pn.*1.2;
                else
%                     disp("高机位捏");
                    pn=pn.*0.8;
                end
            pn(3)=pn(3)+zzz1;
%             plot3(pn(1),pn(2),pn(3),'o')
%             hold on
            cnt=cnt+1;

            pxn=worldCo2jointCo(ii,pn);
            
            try
                qn=jointCo2jointAng(pxn);
                flag=1;
                break
            catch
                continue
            end
        end
        
        assert(flag~=0,"无解");
        end



%p0->px（关节坐标系的目标位置）
jointCo=zeros(3,6);
for i=1:6
    jointCo(:,i)=worldCo2jointCo(i,p0(:,i));
end
%px->q（求三个关节角）
for i=1:6
    try
        k(:,i)=jointCo2jointAng(jointCo(:,i));    %触发关节角限制会有exception
    catch
%         i
        k(:,i)=findNext(i,p0(:,i));
    end
end

end