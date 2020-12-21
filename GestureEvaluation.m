SystemSetup;

%set the joint reference vector
joints=complete_motion(1,:);


states=zeros(5,1);
Mode=zeros(5,1);
thumbCart=zeros(1,4);
indexCart=zeros(1,4);
middleCart=zeros(1,4);
ringCart=zeros(1,4);
pinkieCart=zeros(1,4);

%%%get hull baricentre
[baricentreI,]=getAlphaShapeMorphology(aI);
[baricentreIS,]=getAlphaShapeMorphology(aIS);
[baricentreM,]=getAlphaShapeMorphology(aM);
[baricentreMS,]=getAlphaShapeMorphology(aMS);
[baricentreR,]=getAlphaShapeMorphology(aR);
[baricentreRS,]=getAlphaShapeMorphology(aRS);
[baricentreP,]=getAlphaShapeMorphology(aP);
[baricentrePS,]=getAlphaShapeMorphology(aPS);

jointSolution=zeros(8,20);
cartesianSolution=zeros(8,15);


actual_pose=joints;
thumbJoints=actual_pose(1:4);
thumbJoints(1)=-thumbJoints(1);
indexJoints=actual_pose(5:8);
middleJoints=actual_pose(9:12);
ringJoints=actual_pose(13:16);
pinkieJoints=actual_pose(17:20);


%%%%evalute master finger pose
thumbPose=masterHand.F{1,1}.base*masterThumb_fk(thumbJoints);
indexPose=masterHand.F{1,2}.base*masterIndex_fk(indexJoints);
middlePose=masterHand.F{1,3}.base*masterMiddle_fk(middleJoints);
ringPose=masterHand.F{1,4}.base*masterRing_fk(ringJoints);
pinkiePose=masterHand.F{1,5}.base*masterPinkie_fk(pinkieJoints);

%%%%create pos vector
thumbIn=[thumbPose(1,4),thumbPose(2,4),thumbPose(3,4)];
indexIn=[indexPose(1,4),indexPose(2,4),indexPose(3,4)];
middleIn=[middlePose(1,4),middlePose(2,4),middlePose(3,4)];
ringIn=[ringPose(1,4),ringPose(2,4),ringPose(3,4)];
pinkieIn=[pinkiePose(1,4),pinkiePose(2,4),pinkiePose(3,4)];


%%%%evalute master finger pose
thumbPose=masterHand.F{1,1}.base*masterThumb_fk(thumbJoints);
indexPose=masterHand.F{1,2}.base*masterIndex_fk(indexJoints);
middlePose=masterHand.F{1,3}.base*masterMiddle_fk(middleJoints);
ringPose=masterHand.F{1,4}.base*masterRing_fk(ringJoints);
pinkiePose=masterHand.F{1,5}.base*masterPinkie_fk(pinkieJoints);

%%%%create pos vector
thumbIn=[thumbPose(1,4),thumbPose(2,4),thumbPose(3,4)];
indexIn=[indexPose(1,4),indexPose(2,4),indexPose(3,4)];
middleIn=[middlePose(1,4),middlePose(2,4),middlePose(3,4)];
ringIn=[ringPose(1,4),ringPose(2,4),ringPose(3,4)];
pinkieIn=[pinkiePose(1,4),pinkiePose(2,4),pinkiePose(3,4)];


%%%%evalute master finger control pose w.r.t convex hulls
[Mode_temp,thumbState]=getControlModeThumbNew(thumbIn,aI,aM,aR,aP,aIe,aMe,aRe,aPe,cbr)
Mode(2)=getControlMode(indexIn,aI,aIe,cbr(1));
Mode(3)=getControlMode(middleIn,aM,aMe,cbr(2));
Mode(4)=getControlMode(ringIn,aR,aRe,cbr(3));
Mode(5)=getControlMode(pinkieIn,aP,aPe,cbr(4));


%%%%evalute joint solution for all fingers
jointThumbSol=jThumbMapping(thumbJoints);
jointIndexSol=jIndexMapping(indexJoints);
jointMiddleSol=jMiddleMapping(middleJoints);
jointRingSol=jRingMapping(ringJoints);
jointPinkieSol=jPinkieMapping(pinkieJoints);

%%%%evaluate cartesian pose for joint mapping
ref=[jointThumbSol,jointIndexSol,jointMiddleSol,jointRingSol,jointPinkieSol];
[T_be, T_0e] = slave_fk(ref);
cartThumbSol=[T_0e{1,1}(1,4),T_0e{1,1}(2,4),T_0e{1,1}(3,4)];
cartIndexSol=[T_0e{1,2}(1,4),T_0e{1,2}(2,4),T_0e{1,2}(3,4)];
cartMiddleSol=[T_0e{1,3}(1,4),T_0e{1,3}(2,4),T_0e{1,3}(3,4)];
cartRingSol=[T_0e{1,4}(1,4),T_0e{1,4}(2,4),T_0e{1,4}(3,4)];
cartPinkieSol=[T_0e{1,5}(1,4),T_0e{1,5}(2,4),T_0e{1,5}(3,4)];

%%%%control state evaluation
[Mode,t_state]=setNewControlStatesNew(thumbState,Mode,Mode_temp)

%%%%evaluate cartesian solution for cartesian modes
%%thumb
    if t_state==0
        thumbCart=[0,0,0];
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%sistemare
    if t_state==1
        thumbCart=cartesianThumbEval(thumbPose,baricentreI,baricentreIS,masterHand.F{1,2}.base,slaveHand.F{1,2}.base,slaveHand.F{1,1}.base,scaleI);
    end
    if t_state==2
        thumbCart=cartesianThumbEval(thumbPose,baricentreM,baricentreMS,masterHand.F{1,3}.base,slaveHand.F{1,3}.base,slaveHand.F{1,1}.base,scaleM);
    end
    if t_state==3  
        thumbCart=cartesianThumbEval(thumbPose,baricentreR,baricentreRS,masterHand.F{1,4}.base,slaveHand.F{1,4}.base,slaveHand.F{1,1}.base,scaleR);
    end
    if t_state==4
        thumbCart=cartesianThumbEval(thumbPose,baricentreP,baricentrePS,masterHand.F{1,5}.base,slaveHand.F{1,5}.base,slaveHand.F{1,1}.base,scaleP);
    end

%index
    indexCart=cartesianIndexEval(indexPose,baricentreI,baricentreIS,masterHand.F{1,2}.base,slaveHand.F{1,2}.base,scaleI);

%middle

    middleCart=cartesianIndexEval(middlePose,baricentreM,baricentreMS,masterHand.F{1,3}.base,slaveHand.F{1,3}.base,scaleM);

%ring
    ringCart=cartesianIndexEval(ringPose,baricentreR,baricentreRS,masterHand.F{1,4}.base,slaveHand.F{1,4}.base,scaleR);
%pinkie

    pinkieCart=cartesianIndexEval(pinkiePose,baricentreP,baricentrePS,masterHand.F{1,5}.base,slaveHand.F{1,5}.base,scaleP);



%%%%%final value definition
if Mode(1)>0
    thumbRef=thumbCart;
    thumbRef(1,4)=cartThumbSol(1)*(1-Mode(1))+thumbCart(1,4)*Mode(1);
    thumbRef(2,4)=cartThumbSol(2)*(1-Mode(1))+thumbCart(2,4)*Mode(1);
    thumbRef(3,4)=cartThumbSol(3)*(1-Mode(1))+thumbCart(3,4)*Mode(1);
else
    thumbRef=T_0e{1,1};
end
if Mode(2)>0
    indexRef=indexCart;
    indexRef(1,4)=cartIndexSol(1)*(1-Mode(2))+indexCart(1,4)*Mode(2);
    indexRef(2,4)=cartIndexSol(2)*(1-Mode(2))+indexCart(2,4)*Mode(2);
    indexRef(3,4)=cartIndexSol(3)*(1-Mode(2))+indexCart(3,4)*Mode(2);
else
    indexRef=T_0e{1,2};
end
if Mode(3)>0
    middleRef=middleCart;
    middleRef(1,4)=cartMiddleSol(1)*(1-Mode(3))+middleCart(1,4)*Mode(3);
    middleRef(2,4)=cartMiddleSol(2)*(1-Mode(3))+middleCart(2,4)*Mode(3);
    middleRef(3,4)=cartMiddleSol(3)*(1-Mode(3))+middleCart(3,4)*Mode(3);
else
    middleRef=T_0e{1,3};
end
if Mode(4)>0
    ringRef=ringCart;
    ringRef(1,4)=cartRingSol(1)*(1-Mode(4))+ringCart(1,4)*Mode(4);
    ringRef(2,4)=cartRingSol(2)*(1-Mode(4))+ringCart(2,4)*Mode(4);
    ringRef(3,4)=cartRingSol(3)*(1-Mode(4))+ringCart(3,4)*Mode(4);
else
    ringRef=T_0e{1,4};
end
if Mode(5)>0
    pinkieRef=pinkieCart;
    pinkieRef(1,4)=cartPinkieSol(1)*(1-Mode(5))+pinkieCart(1,4)*Mode(5);
    pinkieRef(2,4)=cartPinkieSol(2)*(1-Mode(5))+pinkieCart(2,4)*Mode(5);
    pinkieRef(3,4)=cartPinkieSol(3)*(1-Mode(5))+pinkieCart(3,4)*Mode(5);
else
    pinkieRef=T_0e{1,5};
end

%%%%finalJoint definition
thumbFinal=thumbIkSlave(thumbRef);
indexFinal=indexIkSlave(indexRef);
middleFinal=middleIkSlave(middleRef);
ringFinal=ringIkSlave(ringRef);
pinkieFinal=pinkieIkSlave(pinkieRef);
thumbOut=[thumbRef(1,4),thumbRef(2,4),thumbRef(3,4)];
indexOut=[indexRef(1,4),indexRef(2,4),indexRef(3,4)];
middleOut=[middleRef(1,4),middleRef(2,4),middleRef(3,4)];
ringOut=[ringRef(1,4),ringRef(2,4),ringRef(3,4)];
pinkieOut=[pinkieRef(1,4),pinkieRef(2,4),pinkieRef(3,4)];


jointSolution(1,:)=[thumbFinal,indexFinal,middleFinal,ringFinal,pinkieFinal];
cartesianSolution(1,:)=[thumbOut,indexOut,middleOut,ringOut,pinkieOut];
ModeSolution(1,:)=Mode(:);
correctThumbRef=(slaveHand.F{1,1}.base*([cartesianSolution(1,1:3),1]'))';
correctThumbRef(4)=[];
correctIndexRef=(slaveHand.F{1,2}.base*([cartesianSolution(1,4:6),1]'))';
correctIndexRef(4)=[];
correctMiddleRef=(slaveHand.F{1,3}.base*([cartesianSolution(1,7:9),1]'))';
correctMiddleRef(4)=[];
correctRingRef=(slaveHand.F{1,4}.base*([cartesianSolution(1,10:12),1]'))';
correctRingRef(4)=[];
correctPinkieRef=(slaveHand.F{1,5}.base*([cartesianSolution(1,13:15),1]'))';
correctPinkieRef(4)=[];
correctCartesianSolution(1,:)=[correctThumbRef,correctIndexRef,correctMiddleRef,correctRingRef,correctPinkieRef];




figure(1)

joints=jointSolution(1,:);
visualHandSlave(slaveHand,joints)

xlabel('x');
ylabel('y');
zlabel('z');
hold on;
indexControlHull.plot('FaceAlpha',0.2,'FaceColor','red','EdgeColor','red','EdgeAlpha',0.2);%,'EdgeColor','blue');
middleControlHull.plot('FaceAlpha',0.2,'FaceColor',[0.9290, 0.6940, 0.1250],'EdgeColor',[0.9290, 0.6940, 0.1250],'EdgeAlpha',0.2)%;,'EdgeColor',[0.9290, 0.6940, 0.1250]);
ringControlHull.plot('FaceAlpha',0.2,'FaceColor',[0, 0.5, 0],'EdgeColor',[0, 0.5, 0],'EdgeAlpha',0.2)%;,'EdgeColor',[0, 0.5, 0]);
pinkieControlHull.plot('FaceAlpha',0.2,'FaceColor',[0.4940, 0.1840, 0.5560],'EdgeColor',[0.4940, 0.1840, 0.5560],'EdgeAlpha',0.2)%;,'EdgeColor',[0.4940, 0.1840, 0.5560]);


