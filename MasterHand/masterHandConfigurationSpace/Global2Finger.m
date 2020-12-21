function [result] = Global2Finger(finger)
    x = round(finger(1))-1;
    y = round(finger(2))-200-1;
    z = round(finger(3))-100-1;
    result=[x,y,z];
end