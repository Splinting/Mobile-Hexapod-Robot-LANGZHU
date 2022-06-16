global xxx1 yyy1 yyy2 zzz1
xo=[xxx1 0,-xxx1,xxx1,0,-xxx1];
yo=[-yyy1, yyy2, -yyy1, yyy1, -yyy2, yyy1];
zo=zzz1;

%% 

origin_jointCo=zeros(3,6);
for i=1:6
origin_jointCo(:,i)=worldCo2jointCo(i,[xo(i);yo(i);zo]);
end

origin_angle=zeros(3,6);
for i=1:6
    [origin_angle(:,i)]=jointCo2jointAng(origin_jointCo(:,i));
end

%机械位形的正负号调整
    origin_angle(1,:)=-origin_angle(1,:);
for i=[2,4,6]  
    origin_angle(2,i)=-origin_angle(2,i);
    origin_angle(3,i)=-origin_angle(3,i);
end

app=zeros(1,18);
for i=1:6
    for j=1:3
        app(1,(i-1)*3+j)=origin_angle(j,i);
    end
end

appp=512+round(app./0.29);

writematrix(appp,'out/originAng.txt','Delimiter',',');