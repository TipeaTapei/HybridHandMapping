function [limits]=getAlphaShapeLimits(shape,base)
    [baricentre,]=getAlphaShapeMorphology(shape);
    [bf,P]=boundaryFacets(shape);
    dim=size(P);
    for i=1:1:dim(1)
        P(i,:)=(P(i,:)-baricentre);
    end
    rot=tform2rotm(base);
    P_temp=(rot\P')';
    min_vector=min(P_temp,[],1);
    max_vector=max(P_temp,[],1);
    limits=[max_vector(1)-min_vector(1);max_vector(2)-min_vector(2);max_vector(3)-min_vector(3)];
end