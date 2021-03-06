function [joints]=cartesianThumbMapping(pose,bariM,bariS,baseM,baseS,scaleFactor,target_hand)
    pose=(pose-bariM)*scaleFactor;
    rotM=tform2rotm(baseM);
    rotS=tform2rotm(baseS);
    pose=(rotM\pose')';
    pose=(rotS*pose')';
    pose=pose+bariS;
    pose=(baseS\[pose, 1]')';
    pose(4)=[];
    %%%%inverse kinematics
    ik = generalizedInverseKinematics;
    ik.RigidBodyTree = target_hand{2};
    ik.ConstraintInputs = {'position','joint'};
    % ik_thumb.SolverParameters.SolutionTolerance = 1e-12;
    poseTg = constraintPositionTarget(target_hand{2}.BodyNames{end});
    poseTg.TargetPosition = pose;
    jointConst = constraintJointBounds(target_hand{2});
    jointConst.Bounds = [-deg2rad(10) deg2rad(10); 0 pi/2; 0 pi/2; 0 pi/2];
    initialguess = homeConfiguration(target_hand{2});
    [configSoln,solnInfo] = ik(initialguess,poseTg,jointConst);
    [q1,q2,q3,q4] = configSoln.JointPosition;
    joints = [q1,q2,q3,q4];
end