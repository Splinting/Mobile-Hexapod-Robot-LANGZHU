%% pitch和roll角  degree
roll=6.43;       % x轴
pitch=-2.32;      % y轴

%% 
app=rotXYAng2jointAng(roll,pitch);
app=reshape(app,1,18);

%机械位形的符号调整
sign=[-1,1,1, -1,-1,-1, -1,1,1, -1,-1,-1, -1,1,1, -1,-1,-1];
app=app.*sign;

appp=512+round(app./0.29);
writematrix(appp,'out/rotTo.txt','Delimiter',',');





