function [modes]=setNewControlStates(stateIn,ModeIn)
    if stateIn==0
        for i=1:5
            modes(i)=0;
        end
    elseif stateIn==1
        if ModeIn(2)==0
            for i=1:5
                modes(i)=0;
            end 
        elseif ModeIn(2)<ModeIn(1)
            modes(1)=ModeIn(2);
            modes(2)=ModeIn(2);
            modes(3)=0;
            modes(4)=0;
            modes(5)=0;
        else
            modes(1)=ModeIn(1);
            modes(2)=ModeIn(1);
            modes(3)=0;
            modes(4)=0;
            modes(5)=0;
        end
    elseif stateIn==2
        if ModeIn(3)==0
            for i=1:5
                modes(i)=0;
            end 
        elseif ModeIn(3)<ModeIn(1)
            modes(1)=ModeIn(3);
            modes(2)=0;
            modes(3)=ModeIn(3);
            modes(4)=0;
            modes(5)=0;
        else
            modes(1)=ModeIn(1);
            modes(2)=0;
            modes(3)=ModeIn(1);
            modes(4)=0;
            modes(5)=0;
        end
    elseif stateIn==3
        if ModeIn(4)==0
            for i=1:5
                modes(i)=0;
            end 
        elseif ModeIn(4)<ModeIn(1)
            modes(1)=ModeIn(4);
            modes(2)=0;
            modes(3)=0;
            modes(4)=ModeIn(4);
            modes(5)=0;
        else
            modes(1)=ModeIn(1);
            modes(2)=0;
            modes(3)=0;
            modes(4)=ModeIn(1);
            modes(5)=0;
        end
    elseif stateIn==4
        if ModeIn(5)==0
            for i=1:5
                modes(i)=0;
            end 
        elseif ModeIn(5)<ModeIn(1)
            modes(1)=ModeIn(2);
            modes(2)=0;
            modes(3)=0;
            modes(4)=0;
            modes(5)=ModeIn(2);
        else
            modes(1)=ModeIn(1);
            modes(2)=0;
            modes(3)=0;
            modes(4)=0;
            modes(5)=ModeIn(1);
        end      
    end
end