function [baricentre,limits]=getAlphaShapeMorphologySlave(shape)
    [bf,P]=boundaryFacets(shape);
    dim = size(P);
    baricentre=mean(P,1);
    min_vector=min(P,[],1);
    max_vector=max(P,[],1);
    limits=[min_vector(1) max_vector(1);min_vector(2) max_vector(2);min_vector(3) max_vector(3)];
end