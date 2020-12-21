function [joints]=pinkieIkSlave(pose)
    load 'ubHand_tree_DH_base.mat' robot_hand;
    target_hand=robot_hand;    
    rotpose=tform2rotm(pose);
    quat=rotm2quat(rotpose);
    position=[pose(1,4),pose(2,4),pose(3,4)];
    %%%%inverse kinematics
    ik = generalizedInverseKinematics;
    ik.RigidBodyTree = target_hand{5};
    ik.ConstraintInputs = {'position','joint','orientation'};
    % ik_thumb.SolverParameters.SolutionTolerance = 1e-12;
    poseTg = constraintPositionTarget(target_hand{5}.BodyNames{end});
    poseTg.TargetPosition = position;
    poseTg.Weights=5;
    jointConst = constraintJointBounds(target_hand{5});
    jointConst.Bounds = [-deg2rad(30) deg2rad(30); 0 pi/2;   0 pi/2; 0 pi/2];
    jointConst.Weights=[100,100,100,100];
    orientTg=constraintOrientationTarget(target_hand{5}.BodyNames{end});
    orientTg.TargetOrientation=quat;
    initialguess = homeConfiguration(target_hand{5});
    [configSoln,solnInfo] = ik(initialguess,poseTg,jointConst,orientTg);
    [q1,q2,q3,q4] = configSoln.JointPosition;
    joints = [q1,q2,q3,q4];
end