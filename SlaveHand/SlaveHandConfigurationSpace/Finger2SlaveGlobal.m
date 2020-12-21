function [result] = Finger2SlaveGlobal(finger)
    x = round(finger(1))+30+1;
    y = round(finger(2))+150+1;
    z = round(finger(3))+1;
    result=[x,y,z];
end