function [baricentre,limits]=getAlphaShapeMorphology(shape)
    [bf,P]=boundaryFacets(shape);
    %dim = size(P);
    %baricentre=mean(P,1);
    min_vector=min(P,[],1);
    max_vector=max(P,[],1);
    limits=[min_vector(1) max_vector(1);min_vector(2) max_vector(2);min_vector(3) max_vector(3)];
    datasize=ceil(max_vector(1)-min_vector(1))*ceil(max_vector(2)-min_vector(2))*ceil(max_vector(3)-min_vector(3));
    datasize=datasize+100;
    x_data=zeros(datasize,1);
    y_data=zeros(datasize,1);
    z_data=zeros(datasize,1);
    count=0;
    for i=min_vector(1):1:max_vector(1)
        for j=min_vector(2):1:max_vector(2)
            for k=min_vector(3):1:max_vector(3)
                if inShape(shape,i,j,k)
                    count=count+1;
                    x_data(count)=i;
                    y_data(count)=j;
                    z_data(count)=k;
                end
            end
        end
    end
    x_data(count+1:datasize)=[];
    y_data(count+1:datasize)=[];
    z_data(count+1:datasize)=[];
    baricentre(1)=mean(x_data);
    baricentre(2)=mean(y_data);
    baricentre(3)=mean(z_data);
end