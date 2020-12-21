%Mapping Evaluator for a series of joint trajectory references
SystemSetup;

%activate reevaluation of the the joint trajectory for the slave (set 0 to 
%graphically show the previously evaluated one
evaluate_trajectory=0;

if evaluate_trajectory

    %define the reference matrix m x n with m points and n joints 
    full_control_vector=complete_motion_ext;

    serie_dim=size(full_control_vector);

    thumbJoints=[0,0,0,0];
    indexJoints=[0,0,0,0];
    middleJoints=[0,0,0,0];
    ringJoints=[0,0,0,0];
    pinkieJoints=[0,0,0,0];
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

    jointSolution=zeros(length(full_control_vector),20);
    cartesianSolution=zeros(length(full_control_vector),15);
    ModeSolution=zeros(length(full_control_vector),5);
    correctCartesianSolution=zeros(length(full_control_vector),15);

    %cycle over points
    for k=1:serie_dim(1)
        actual_pose=full_control_vector(k,:);
        thumbJoints=actual_pose(1:4);
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
                thumbCart=eye(4);
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


        jointSolution(k,:)=[thumbFinal,indexFinal,middleFinal,ringFinal,pinkieFinal];
        cartesianSolution(k,:)=[thumbOut,indexOut,middleOut,ringOut,pinkieOut];
        ModeSolution(k,:)=Mode(:);
        correctThumbRef=(slaveHand.F{1,1}.base*([cartesianSolution(k,1:3),1]'))';
        correctThumbRef(4)=[];
        correctIndexRef=(slaveHand.F{1,2}.base*([cartesianSolution(k,4:6),1]'))';
        correctIndexRef(4)=[];
        correctMiddleRef=(slaveHand.F{1,3}.base*([cartesianSolution(k,7:9),1]'))';
        correctMiddleRef(4)=[];
        correctRingRef=(slaveHand.F{1,4}.base*([cartesianSolution(k,10:12),1]'))';
        correctRingRef(4)=[];
        correctPinkieRef=(slaveHand.F{1,5}.base*([cartesianSolution(k,13:15),1]'))';
        correctPinkieRef(4)=[];
        correctCartesianSolution(k,:)=[correctThumbRef,correctIndexRef,correctMiddleRef,correctRingRef,correctPinkieRef];

    end
    figure(12)
    %plot of all fingers along x,y,z
    subplot(4,1,1)
    plot(correctCartesianSolution(:,1));
    hold on
    plot(correctCartesianSolution(:,4));
    plot(correctCartesianSolution(:,7));
    plot(correctCartesianSolution(:,10));
    plot(correctCartesianSolution(:,13));
    subplot(4,1,2)
    plot(correctCartesianSolution(:,2));
    hold on
    plot(correctCartesianSolution(:,5));
    plot(correctCartesianSolution(:,8));
    plot(correctCartesianSolution(:,11));
    plot(correctCartesianSolution(:,14));
    subplot(4,1,3)
    plot(correctCartesianSolution(:,3));
    hold on
    plot(correctCartesianSolution(:,6));
    plot(correctCartesianSolution(:,9));
    plot(correctCartesianSolution(:,12));
    plot(correctCartesianSolution(:,15));
else
    load('paper_trajectory.mat')
    full_control_vector=complete_motion_ext;
end
figure(1)
view(114.42,15.92)
xlabel('X');
ylabel('Y');
zlabel('Z');
title('master hand')
   

figure(2)
view(114.42,15.92)
xlabel('X');
ylabel('Y');
zlabel('Z');
title('slave_hand')
i=1;
while i<length(complete_motion_ext)
   fff1 = figure(1)
   movegui(fff1,'west');
   visualMasterHand(masterHand,complete_motion_ext(i,:));
   view(114.42,15.92)
   title('Master Hand Motion')
   drawnow
   fff2 = figure(2)
   movegui(fff2,'east');
   visualHandSlave(slaveHand,jointSolution(i,:));
   view(114.42,15.92)
   title('Slave Hand Motion Resulting From The Mapping')
   if sum(ModeSolution(i:i+4,:),'all')>0
       i=i+1;
   else
       i=i+5;
   end
end
