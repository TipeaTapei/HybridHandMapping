function [jointRef]=jointThumbMapping(joints)

    %%%%%%%thumb mapping%%%%%%%%%%%
    jointRef(1)=joints(4);
    jointRef(2)=joints(1)+deg2rad(90);
    jointRef(3)=joints(2);
    jointRef(4)=joints(3);
end