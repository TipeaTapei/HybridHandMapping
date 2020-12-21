function [mode] = getControlMode(pose,shape,shapeExt,coeff)
    if inShape(shape,pose(1),pose(2),pose(3))==1
        mode=1;
    else
        if inShape(shapeExt,pose(1),pose(2),pose(3))
            mode = cos(rbfinterp(pose', coeff)-pi/2);
        else
            mode = 0;
        end
    end
end