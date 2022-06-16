ap=trace_jointAng;

for i=1:6
    ap{i}(1,:)=-ap{i}(1,:);
end

for i=[2 4 6]
    ap{i}(2,:)=-ap{i}(2,:);
    ap{i}(3,:)=-ap{i}(3,:);
end

app=zeros(1,6*3*nn);
for i=1:nn
    app(18*(i-1)+1)=ap{1}(1,i);
    app(18*(i-1)+2)=ap{1}(2,i);
    app(18*(i-1)+3)=ap{1}(3,i);
    app(18*(i-1)+4)=ap{2}(1,i);
    app(18*(i-1)+5)=ap{2}(2,i);
    app(18*(i-1)+6)=ap{2}(3,i);
    app(18*(i-1)+7)=ap{3}(1,i);
    app(18*(i-1)+8)=ap{3}(2,i);
    app(18*(i-1)+9)=ap{3}(3,i);
    app(18*(i-1)+10)=ap{4}(1,i);
    app(18*(i-1)+11)=ap{4}(2,i);
    app(18*(i-1)+12)=ap{4}(3,i);
    app(18*(i-1)+13)=ap{5}(1,i);
    app(18*(i-1)+14)=ap{5}(2,i);
    app(18*(i-1)+15)=ap{5}(3,i);
    app(18*(i-1)+16)=ap{6}(1,i);
    app(18*(i-1)+17)=ap{6}(2,i);
    app(18*(i-1)+18)=ap{6}(3,i);
    
end
appp=512+round(app./0.29);
writematrix(appp,'out/fullGait.txt','Delimiter',',');

