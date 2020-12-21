function [shape]=getAlphaExternalLayer(shapeIn,factor)
    [baricentre,]=getAlphaShapeMorphology(shapeIn);
    [bf,P]=boundaryFacets(shapeIn);
    dim=size(P);
    for i=1:1:dim(1)
        P(i,:)=(P(i,:)-baricentre)*factor+baricentre;
    end
    shape=alphaShape(P(:,1),P(:,2),P(:,3));
    shape.Alpha=inf;
end