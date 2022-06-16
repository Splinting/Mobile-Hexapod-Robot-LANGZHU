function leg=genLeg(THETA,X,Y,Z,name)
        %(X,Y,Z)=base coordinate
        %theta in radian
global l1 l2 l3;

L(1)=Link('revolute','d',0,'a',l1,'alpha',pi/2,'offset',THETA);
L(2)=Link('revolute','d',0,'a',l2,'alpha',0,'offset',pi/2);
L(3)=Link('revolute','d',0,'a',l3,'alpha',0,'offset',-169.09/180*pi);

leg=SerialLink(L,'name',['leg',name]);
leg.base=transl(X,Y,Z);

% view(3);
% leg.teach;  
end