extFactor=1.1;

[indexControlHull,scaleI]=getMasterToSlaveHull(aI,aIS,masterHand.F{1,2}.base,slaveHand.F{1,2}.base);
[middleControlHull,scaleM]=getMasterToSlaveHull(aM,aMS,masterHand.F{1,3}.base,slaveHand.F{1,3}.base);
[ringControlHull,scaleR]=getMasterToSlaveHull(aR,aRS,masterHand.F{1,4}.base,slaveHand.F{1,4}.base);
[pinkieControlHull,scaleP]=getMasterToSlaveHull(aP,aPS,masterHand.F{1,5}.base,slaveHand.F{1,5}.base);

figure(7)
joints=zeros(1,20);
visualHandSlave(slaveHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
grid on;
hold on;

aIS.plot('FaceAlpha','0.1');
aMS.plot('FaceAlpha','0.1');
aRS.plot('FaceAlpha','0.1');
aPS.plot('FaceAlpha','0.1');

indexControlHull.plot;
middleControlHull.plot;
ringControlHull.plot;
pinkieControlHull.plot;

%%%evaluate external hulls

indexExtHull=getAlphaExternalLayer(indexControlHull,extFactor);
middleExtHull=getAlphaExternalLayer(middleControlHull,extFactor);
ringExtHull=getAlphaExternalLayer(ringControlHull,extFactor);
pinkieExtHull=getAlphaExternalLayer(pinkieControlHull,extFactor);