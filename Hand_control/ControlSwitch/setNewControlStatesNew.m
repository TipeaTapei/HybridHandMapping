function [modes,state]=setNewControlStatesNew(stateIn,ModeIn,ModeThumb)
    
    for i=1:5
        modes(i)=0;
        state=0;
    end

    if stateIn(2)==1
        if ModeIn(2)==0
            modes(2)=0; 
        elseif ModeIn(2)<ModeThumb(2)
            if modes(1)<ModeIn(2)
                state=1;
                modes(1)=ModeIn(2);
            end
            modes(2)=ModeIn(2);
        else
            if modes(1)<ModeThumb(2)
                state=1;
                modes(1)=ModeThumb(2);
            end
            modes(2)=ModeThumb(2);
        end
    end
    if stateIn(3)==1
        if ModeIn(3)==0
            modes(3)=0;
        elseif ModeIn(3)<ModeThumb(3)
            if modes(1)<ModeIn(3)
                state=2;
                modes(1)=ModeIn(3);
            end
            modes(3)=ModeIn(3);
        else
            if modes(1)<ModeThumb(3)
                state=2;
                modes(1)=ModeThumb(3);
            end
            modes(3)=ModeThumb(3);
        end
    end
    if stateIn(4)==1
        if ModeIn(4)==0
            modes(4)=0;
        elseif ModeIn(4)<ModeThumb(4)
            if modes(1)<ModeIn(4)
                state=3;
                modes(1)=ModeIn(4);
            end
            modes(4)=ModeIn(4);
        else
            if modes(1)<ModeThumb(4)
                state=3;
                modes(1)=ModeThumb(4);
            end
            modes(4)=ModeThumb(4);
        end
    end
    if stateIn(5)==1
        if ModeIn(5)==0
            modes(5)=0;
        elseif ModeIn(5)<ModeThumb(5)
            if modes(1)<ModeIn(5)
                state=4;
                modes(1)=ModeIn(5);
            end
            modes(5)=ModeIn(5);
        else
            if modes(1)<ModeThumb(5)
                state=4;
                modes(1)=ModeThumb(5);
            end
            modes(5)=ModeThumb(5);
        end      
    end
end