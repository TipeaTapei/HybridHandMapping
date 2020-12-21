function [alphaShapeSpace] = getAlphaShapeSpaceSlave(fingerspace,spacedim)
    elements = nnz(fingerspace)
    xvector=zeros(elements,1);
    yvector=zeros(elements,1);
    zvector=zeros(elements,1);
    counter=1;
    for x=1:1:spacedim(1)
        for y=1:1:spacedim(2)
            for z=1:1:spacedim(3)
                if fingerspace(x,y,z)==1
                    result=Global2SlaveFinger([x,y,z]);
                    xvector(counter)=result(1);
                    yvector(counter)=result(2);
                    zvector(counter)=result(3);
                    counter=counter+1;
                end
            end
        end
    end
    alphaShapeSpace=alphaShape(xvector,yvector,zvector);
    
end