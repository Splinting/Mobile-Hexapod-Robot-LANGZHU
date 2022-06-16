ap=trace_jointAng;

%机械位形的正负号调整
for i=1:6
    ap{i}(1,:)=-ap{i}(1,:);
end

for i=[2,4,6]
    ap{i}(2,:)=-ap{i}(2,:);
    ap{i}(3,:)=-ap{i}(3,:);
end

N=2;        %输出腿数量
leg=[2,6];  %输出腿号

app=zeros(1,N*3*nn);
for i=1:nn
    for p=1:N
        for j=1:3
            app(3*N*(i-1)+j+(p-1)*3)=ap{leg(p)}(j,i);
        end
    end

end
appp=512+round(app./0.29);
writematrix(appp,'out/singleGait.txt','Delimiter',',');

