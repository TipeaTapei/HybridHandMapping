function [target_pose]=cartesianIndexEval(poseIn,bariM,bariS,baseM,baseS,scaleFactor)
    indexIn=[poseIn(1,4),poseIn(2,4),poseIn(3,4)];
    pose=(indexIn-bariM)*scaleFactor;
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
end