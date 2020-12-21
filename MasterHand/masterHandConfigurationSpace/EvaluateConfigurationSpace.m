
theta_step=0.03;
GlobalSpaceThumb = zeros(globalDim(1),globalDim(2),globalDim(3));
GlobalSpaceIndex = zeros(globalDim(1),globalDim(2),globalDim(3));
GlobalSpaceMiddle = zeros(globalDim(1),globalDim(2),globalDim(3));
GlobalSpaceRing = zeros(globalDim(1),globalDim(2),globalDim(3));
GlobalSpacePinkie = zeros(globalDim(1),globalDim(2),globalDim(3));

disp("evaluating thumb configuration space");

for x1=ThumbLimit_theta0(1):theta_step:ThumbLimit_theta0(2)
    for x2=ThumbLimit_theta1(1):theta_step:ThumbLimit_theta1(2)
        for x3=ThumbLimit_theta2(1):theta_step:ThumbLimit_theta2(2)
            for x4=ThumbLimit_theta3(1):theta_step:ThumbLimit_theta3(2) 
                joints=[x1,x2,x3,x4];
                sol = masterHand.F{1,1}.base*masterThumb_fk(joints);
                solution=[sol(1,4),sol(2,4),sol(3,4)];
                space=Finger2Global(solution);
                if space(1)<1
                    space(1)=1;
                end
                GlobalSpaceThumb(space(1),space(2),space(3))=1;
            end
        end
    end
end
disp("done")

disp("evaluating index configuration space");

for x1=IndexLimit_theta0(1):theta_step:IndexLimit_theta0(2)
    for x2=IndexLimit_theta1(1):theta_step:IndexLimit_theta1(2)
        for x3=IndexLimit_theta2(1):theta_step:IndexLimit_theta2(2)
            for x4=IndexLimit_theta3(1):theta_step:IndexLimit_theta3(2) 
                joints=[x1,x2,x3,x4];
                sol = masterHand.F{1,2}.base*masterIndex_fk(joints);
                solution=[sol(1,4),sol(2,4),sol(3,4)];
                space=Finger2Global(solution);
                if space(1)<1
                    space(1)=1;
                end
                GlobalSpaceIndex(space(1),space(2),space(3))=1;
            end
        end
    end
end
disp("done")

disp("evaluating middle configuration space");
for x1=MiddleLimit_theta0(1):theta_step:MiddleLimit_theta0(2)
    for x2=MiddleLimit_theta1(1):theta_step:MiddleLimit_theta1(2)
        for x3=MiddleLimit_theta2(1):theta_step:MiddleLimit_theta2(2)
            for x4=MiddleLimit_theta3(1):theta_step:MiddleLimit_theta3(2) 
                joints=[x1,x2,x3,x4];
                sol = masterHand.F{1,3}.base*masterMiddle_fk(joints);
                solution=[sol(1,4),sol(2,4),sol(3,4)];
                space=Finger2Global(solution);
                if space(1)<1
                    space(1)=1;
                end
                GlobalSpaceMiddle(space(1),space(2),space(3))=1;
            end
        end
    end
end
disp("done")

disp("evaluating ring configuration space");

for x1=RingLimit_theta0(1):theta_step:RingLimit_theta0(2)
    for x2=RingLimit_theta1(1):theta_step:RingLimit_theta1(2)
        for x3=RingLimit_theta2(1):theta_step:RingLimit_theta2(2)
            for x4=RingLimit_theta3(1):theta_step:RingLimit_theta3(2) 
                joints=[x1,x2,x3,x4];
                sol = masterHand.F{1,4}.base*masterRing_fk(joints);
                solution=[sol(1,4),sol(2,4),sol(3,4)];
                space=Finger2Global(solution);
                if space(1)<1
                    space(1)=1;
                end
                GlobalSpaceRing(space(1),space(2),space(3))=1;
            end
        end
    end
end
disp("done")

disp("evaluating pinkie configuration space");
for x1=PinkieLimit_theta0(1):theta_step:PinkieLimit_theta0(2)
    for x2=PinkieLimit_theta1(1):theta_step:PinkieLimit_theta1(2)
        for x3=PinkieLimit_theta2(1):theta_step:PinkieLimit_theta2(2)
            for x4=PinkieLimit_theta3(1):theta_step:PinkieLimit_theta3(2) 
                joints=[x1,x2,x3,x4];
                sol = masterHand.F{1,5}.base*masterPinkie_fk(joints);
                solution=[sol(1,4),sol(2,4),sol(3,4)];
                space=Finger2Global(solution);
                if space(1)<1
                    space(1)=1;
                end
                GlobalSpacePinkie(space(1),space(2),space(3))=1;
            end
        end
    end
end
disp("done")




            
            
            