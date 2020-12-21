function [T_be,T_0e] = fkineHand(teta)
    %fkineHand: take as input the transformation of the finger bases, the
    %DH parameters and theta, and return the transformations of each fingertip of the hand
    %both with respect to the hand base (T_be) and to the finger bases (T_0e)
    load 'ubHand_tree_DH_base.mat' base;
    load 'ubHand_tree_DH_base.mat' DHpars;
    teta = rad2deg(teta);
    
    temp = [teta(1:4);teta(5:8);teta(9:12);teta(13:16);teta(17:20)];
    teta=temp;
    n_fingers = size(DHpars);
    n_fingers = n_fingers(1,2);

    for i = 1:n_fingers                %per ogni dito
        A = eye(4);
        n_link = size(DHpars{i});
        n_link = n_link(1,1);
        for j = 1:n_link                 %per ogni link
            ct = cosd(teta(i,j));
            st = sind(teta(i,j));
            ca = cosd(rad2deg(DHpars{i}(j,1)));
            sa = sind(rad2deg(DHpars{i}(j,1)));

            A = A * [ct	-st*ca	st*sa   DHpars{i}(j,2)*ct ;
                     st  ct*ca  -ct*sa  DHpars{i}(j,2)*st ;
                     0   sa     ca      DHpars{i}(j,3)    ;
                     0   0      0       1         ];
            if i ~= 1
                T_0e{i} = A;
            else
                T_0e{i} = A;
            end
            
        end
        T_be{i} = base{i}*T_0e{i};
    end
end

