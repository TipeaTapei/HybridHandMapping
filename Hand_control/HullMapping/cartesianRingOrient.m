function [joints]=cartesianRingOrient(poseIn,bariM,bariS,baseM,baseS,scaleFactor,target_hand)
    ringIn=[poseIn(1,4),poseIn(2,4),poseIn(3,4)];
    pose=(ringIn-bariM)*scaleFactor;
    rotM=tform2rotm(baseM);
    rotS=tform2rotm(baseS);
    pose=(rotM\pose')';
    pose=(rotS*pose')';
    pose=pose+bariS;
    pose=(baseS\[pose, 1]')';
    pose(4)=[];
    rotpose=tform2rotm(poseIn);
    rotpose=rotM\rotpose;
    target_pose=rotm2tform(rotpose);
    target_pose(1,4)=pose(1);
    target_pose(2,4)=pose(2);
    target_pose(3,4)=pose(3);
    quat=rotm2quat(rotpose);
    %%%%inverse kinematics
    ik = generalizedInverseKinematics;
    ik.RigidBodyTree = target_hand{4};
    ik.ConstraintInputs = {'position','joint','orientation'};
    % ik_thumb.SolverParameters.SolutionTolerance = 1e-12;
    poseTg = constraintPositionTarget(target_hand{4}.BodyNames{end});
    poseTg.TargetPosition = pose;
    poseTg.Weights=5;
    jointConst = constraintJointBounds(target_hand{4});
    jointConst.Bounds = [-deg2rad(30) deg2rad(30); -pi/2 pi/2;   0 pi/2; 0 pi/2];
    jointConst.Weights=[100,100,100,100];
    orientTg=constraintOrientationTarget(target_hand{4}.BodyNames{end});
    orientTg.TargetOrientation=quat;
    initialguess = homeConfiguration(target_hand{4});
    [configSoln,solnInfo] = ik(initialguess,poseTg,jointConst,orientTg);
    [q1,q2,q3,q4] = configSoln.JointPosition;
    joints = [q1,q2,q3,q4];
end