function [controlSphere] = getSphere(fingerspace,thumbspace,globalDim)
    controlSphere=zeros(globalDim(1),globalDim(2),globalDim(3));
    for x=1:1:globalDim(1)
        for y=1:1:globalDim(2)
            for z=1:1:globalDim(3)
                if fingerspace(x,y,z)==1
                    if thumbspace(x,y,z)==1
                        controlSphere(x,y,z)=1;
                    end
                end
            end
        end
    end
end