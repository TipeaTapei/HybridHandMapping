function [joints]=cartesianThumbMapping(pose,bariM,bariS,baseM,baseS,baseT,scaleFactor,target_hand)
    pose=(pose-bariM)*scaleFactor;
    rotM=tform2rotm(baseM);
    rotS=tform2rotm(baseS);
    rotT=tform2rotm(baseT);
    pose=(rotM\pose')';
    pose=(rotS*pose')';
    pose=pose+bariS;
    pose=(baseT\[pose,1]')';
    pose(4)=[];
    %%%%inverse kinematics
    ik = generalizedInverseKinematics;
    ik.RigidBodyTree = target_hand{1};
    ik.ConstraintInputs = {'position','joint'};
    % ik_thumb.SolverParameters.SolutionTolerance = 1e-12;
    poseTg = constraintPositionTarget(target_hand{1}.BodyNames{end});
    poseTg.TargetPosition = pose;
    jointConst = constraintJointBounds(target_hand{1});
    jointConst.Bounds = [0 pi/2; -pi pi; 0 pi/2; 0 pi/2];
    initialguess = homeConfiguration(target_hand{1});
    [configSoln,solnInfo] = ik(initialguess,poseTg,jointConst);
    [q1,q2,q3,q4] = configSoln.JointPosition;
    joints = [q1,q2,q3,q4];
end