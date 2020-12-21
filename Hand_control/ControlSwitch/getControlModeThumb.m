function [mode,state] = getControlModeThumb(pose,aI,aM,aR,aP,aIS,aMS,aRS,aPS,coeff)
    if inShape(aI,pose(1),pose(2),pose(3))
        mode=1;
        state=1;
    elseif inShape(aIS,pose(1),pose(2),pose(3))
            mode = rbfinterp(pose', coeff(1));
            state=1;
    elseif inShape(aM,pose(1),pose(2),pose(3))
            mode = 1;
            state=2;
    elseif inShape(aMS,pose(1),pose(2),pose(3))
            mode = rbfinterp(pose', coeff(2));
            state=2;
    elseif inShape(aR,pose(1),pose(2),pose(3))
            mode = 1;
            state=3;
    elseif inShape(aRS,pose(1),pose(2),pose(3))
            mode = rbfinterp(pose', coeff(3));
            state=3;
    elseif inShape(aP,pose(1),pose(2),pose(3))
            mode = 1;
            state=4;
    elseif inShape(aPS,pose(1),pose(2),pose(3))
            mode = rbfinterp(pose', coeff(4));
            state=4;
    else 
        mode=0;
        state=0;
    end
end