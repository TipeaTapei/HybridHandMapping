function []=plotGraphGlobal(fingerspace,spacedim)

elements = nnz(fingerspace)
xvector=zeros(elements,1);
yvector=zeros(elements,1);
zvector=zeros(elements,1);
counter=1;



for x=1:1:spacedim(1)
    for y=1:1:spacedim(2)
        for z=1:1:spacedim(3)
            if fingerspace(x,y,z)==1
                result=Global2Finger([x,y,z]);
                xvector(counter)=result(1);
                yvector(counter)=result(2);
                zvector(counter)=result(3);
                counter=counter+1;
            end
        end
    end
end
hold on
scatter3(xvector,yvector,zvector)
xlabel('x')
ylabel('y')
zlabel('z')
axis([0 70 -30 40 0 160])
grid on           
end