%definition of master finger configuration spaces
EvaluateConfigurationSpace

%set up of hulls transition factor
shapeFactor=1.2;

%Visualization of master hand configuration spaces
figure(1)
joints=zeros(1,20);
visualMasterHand(masterHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
hold on;

plotGraphGlobal(GlobalSpaceIndex,globalDim);
plotGraphGlobal(GlobalSpaceMiddle,globalDim);
plotGraphGlobal(GlobalSpaceRing,globalDim);
plotGraphGlobal(GlobalSpacePinkie,globalDim);
plotGraphGlobal(GlobalSpaceThumb,globalDim);

%Visualization of global hand hulls
indexControlSphere=getSphere(GlobalSpaceIndex,GlobalSpaceThumb,globalDim);
middleControlSphere=getSphere(GlobalSpaceMiddle,GlobalSpaceThumb,globalDim);
ringControlSphere=getSphere(GlobalSpaceRing,GlobalSpaceThumb,globalDim);
pinkieControlSphere=getSphere(GlobalSpacePinkie,GlobalSpaceThumb,globalDim);

figure(2)
visualMasterHand(masterHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
hold on;

plotGraphGlobal(indexControlSphere,globalDim);
plotGraphGlobal(middleControlSphere,globalDim);
plotGraphGlobal(ringControlSphere,globalDim);
plotGraphGlobal(pinkieControlSphere,globalDim);

%evaluate and plot alpha shapes
aI=getAlphaShapeSpace(indexControlSphere,globalDim);
aI.Alpha=inf;
aM=getAlphaShapeSpace(middleControlSphere,globalDim);
aM.Alpha=inf;
aR=getAlphaShapeSpace(ringControlSphere,globalDim);
aR.Alpha=inf;
aP=getAlphaShapeSpace(pinkieControlSphere,globalDim);
aP.Alpha=inf;

figure(3)
visualMasterHand(masterHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
hold on;
aI.plot;
aM.plot;
aR.plot;
aP.plot;

%%%evaluate external alpha shape
aIe=getAlphaExternalLayer(aI,shapeFactor);
aMe=getAlphaExternalLayer(aM,shapeFactor);
aRe=getAlphaExternalLayer(aR,shapeFactor);
aPe=getAlphaExternalLayer(aP,shapeFactor);

%%%evaluate interpolation zone 
cbr(1)=getCbrTransition(aI,aIe);
cbr(2)=getCbrTransition(aM,aMe);
cbr(3)=getCbrTransition(aR,aRe);
cbr(4)=getCbrTransition(aP,aPe);
