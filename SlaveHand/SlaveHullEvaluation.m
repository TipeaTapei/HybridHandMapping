%definition of master finger configuration spaces
EvaluateConfigurationSpaceSlave

figure(4)
joints=zeros(1,20);
visualHandSlave(slaveHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
grid on;

hold on;

%plot global hand spaces
plotGraphGlobalSlave(GlobalSlaveSpaceIndex,globalSlaveDim);
plotGraphGlobalSlave(GlobalSlaveSpaceMiddle,globalSlaveDim);
plotGraphGlobalSlave(GlobalSlaveSpaceRing,globalSlaveDim);
plotGraphGlobalSlave(GlobalSlaveSpacePinkie,globalSlaveDim);
plotGraphGlobalSlave(GlobalSlaveSpaceThumb,globalSlaveDim);

%plot global hand hulls
indexSlaveControlSphere = getSphereSlave(GlobalSlaveSpaceIndex,GlobalSlaveSpaceThumb,globalSlaveDim);
middleSlaveControlSphere = getSphereSlave(GlobalSlaveSpaceMiddle,GlobalSlaveSpaceThumb,globalSlaveDim);
ringSlaveControlSphere = getSphereSlave(GlobalSlaveSpaceRing,GlobalSlaveSpaceThumb,globalSlaveDim);
pinkieSlaveControlSphere = getSphereSlave(GlobalSlaveSpacePinkie,GlobalSlaveSpaceThumb,globalSlaveDim);

figure(5)
visualHandSlave(slaveHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
hold on;

plotGraphGlobalSlave(indexSlaveControlSphere,globalSlaveDim);
plotGraphGlobalSlave(middleSlaveControlSphere,globalSlaveDim);
plotGraphGlobalSlave(ringSlaveControlSphere,globalSlaveDim);
plotGraphGlobalSlave(pinkieSlaveControlSphere,globalSlaveDim);

%evaluate and plot alpha shapes
aIS=getAlphaShapeSpaceSlave(indexSlaveControlSphere,globalSlaveDim);
aIS.Alpha=inf;
aMS=getAlphaShapeSpaceSlave(middleSlaveControlSphere,globalSlaveDim);
aMS.Alpha=inf;
aRS=getAlphaShapeSpaceSlave(ringSlaveControlSphere,globalSlaveDim);
aRS.Alpha=inf;
aPS=getAlphaShapeSpaceSlave(pinkieSlaveControlSphere,globalSlaveDim);
aPS.Alpha=inf;

figure(6)
visualHandSlave(slaveHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
hold on;
aIS.plot;
aMS.plot;
aRS.plot;
aPS.plot;
