function [coeff]=getCbrTransition(shape,extShape)
    [bf,P]=boundaryFacets(shape);
    [bf,Pex]=boundaryFacets(extShape);
    dim=size(P);
    dim_ext=size(Pex);
    sol1=zeros(1,dim_ext(1));
    sol2=ones(1,dim(1));
    sol=cat(2,sol1,sol2);
    P_tot=(cat(2,Pex',P'));
    coeff = rbfcreate(P_tot, sol);
end