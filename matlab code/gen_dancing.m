%% 
mag=20; %旋转的角度（+-mag）
n=40;

%% 
% x=zeros(1,40);
% x(1:5)=mag;
% x(6:15)=tpoly(mag,-mag,10,0,0);
% x(16:25)=-mag;
% x(26:35)=tpoly(-mag,mag,10,0,0);
% x(36:40)=mag;
% plot(x)
% hold on
% 
% y=zeros(1,40);
% s(1:10)=tpoly(-mag,mag,10,0,0);
% y(1:5)=s(6:10);
% y(6:15)=mag*ones(1,10);
% y(16:25)=tpoly(mag,-mag,10,0,0);
% y(26:35)=-mag*ones(1,10);
% y(36:40)=s(1:5);
% plot(y)

%% 
tt=0:(2*pi)/n:2*pi;
x=mag.*cos(tt);
y=mag.*sin(tt);

% figure(2)
% plot(x)
% hold on
% plot(y)
%%

%机械位形的符号调整
sign=[-1,1,1, -1,-1,-1, -1,1,1, -1,-1,-1, -1,1,1, -1,-1,-1];

out=zeros(1,18*40);
for i=1:40
    k=rotXYAng2jointAng(x(i),y(i));
    out((1+(i-1)*18):(18+(i-1)*18))=reshape(k,1,18).*sign;
end

out=512+round(out./0.29);
writematrix(out,'out/dancing.txt','Delimiter',',');