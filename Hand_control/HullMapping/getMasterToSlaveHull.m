%This script returns the Alpha shape hull for a finger, scaled on the x
%direction of the slave hull
function [shape,scalingFactor]=getMasterToSlaveHull(shapeMaster,shapeSlave,baseM,baseS)
    limitsMaster=getAlphaShapeLimits(shapeMaster,baseM);
    limitsSlave=getAlphaShapeLimits(shapeSlave,baseS);
    scalingFactor=limitsSlave(3)/limitsMaster(3);
    [baricentreM,]=getAlphaShapeMorphology(shapeMaster);
    [baricentreS,]=getAlphaShapeMorphology(shapeSlave);
    [bf,P]=boundaryFacets(shapeMaster);
    dim=size(P);
    for i=1:1:dim(1)
        P(i,:)=(P(i,:)-baricentreM)*scalingFactor;
    end
    rotM=tform2rotm(baseM);
    rotS=tform2rotm(baseS);
    NewP=(rotM\P')';
    NewP=(rotS*NewP')';
    dim2=size(NewP);
    for i=1:1:dim2(1)
        NewP(i,:)=NewP(i,:)+baricentreS;
    end
    shape=alphaShape(NewP(:,1),NewP(:,2),NewP(:,3));
    shape.Alpha=inf;

end