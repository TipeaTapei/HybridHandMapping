function [mode,state] = getControlModeThumbNew(pose,aI,aM,aR,aP,aIS,aMS,aRS,aPS,coeff)
    mode=zeros(5,1);
    state=zeros(5,1);
    if inShape(aI,pose(1),pose(2),pose(3))
        mode(2)=1;
        state(2)=1;
    elseif inShape(aIS,pose(1),pose(2),pose(3))
            mode(2) = cos(rbfinterp(pose', coeff(1))-pi/2);
            state(2)=1;
    end
    if inShape(aM,pose(1),pose(2),pose(3))
            mode(3) = 1;
            state(3)=1;
    elseif inShape(aMS,pose(1),pose(2),pose(3))
            mode(3) = cos(rbfinterp(pose', coeff(2))-pi/2);
            state(3)=1;
    end
    if inShape(aR,pose(1),pose(2),pose(3))
            mode(4) = 1;
            state(4)=1;
    elseif inShape(aRS,pose(1),pose(2),pose(3))
            mode(4) = cos(rbfinterp(pose', coeff(3))-pi/2);
            state(4)=1;
    end
    if inShape(aP,pose(1),pose(2),pose(3))
            mode(5) = 1;
            state(5)=1;
    elseif inShape(aPS,pose(1),pose(2),pose(3))
            mode(5) = cos(rbfinterp(pose', coeff(4))-pi/2);
            state(5)=1;
    end
end