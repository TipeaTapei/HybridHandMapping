function [finger_transform] = ParaFingerFkThumb(joints)
    Q_1 = joints(1);
    Q_2 = joints(2); 
    Q_3 = joints(3);
    Q_4 = joints(4);
    visual_scaling = 1;

    % parametri D-H
    a = [0 0 0 0];
    % a(1) = 0;
    % a(2) = 47.87/visual_scaling;
    % a(3) = 29.25/visual_scaling;
    % a(4) = 29.2/visual_scaling;
    a(1) = 0;
    a(2) = 35/visual_scaling;
    a(3) = 30/visual_scaling;
    a(4) = 20/visual_scaling;

    d=[0 0 0 0];
    d(1)=0;
    d(2)=0;
    d(3)=0;
    d(4)=0;

    alpha=[0 0 0 0];
    alpha(1)=pi/2;
    alpha(2)=0;
    alpha(3)=-pi/2;
    alpha(4)=0;
    
    T_01 = [cos(Q_1) -sin(Q_1) * cos(alpha(1)) sin(Q_1) * sin(alpha(1)) a(1) * cos(Q_1); sin(Q_1) cos(Q_1) * cos(alpha(1)) -cos(Q_1) * sin(alpha(1)) a(1) * sin(Q_1); 0 sin(alpha(1)) cos(alpha(1)) d(1); 0 0 0 1;];
    T_12 = [cos(Q_2) -sin(Q_2) * cos(alpha(2)) sin(Q_2) * sin(alpha(2)) a(2) * cos(Q_2); sin(Q_2) cos(Q_2) * cos(alpha(2)) -cos(Q_2) * sin(alpha(2)) a(2) * sin(Q_2); 0 sin(alpha(2)) cos(alpha(2)) d(2); 0 0 0 1;];
    T_23 = [cos(Q_3) -sin(Q_3) * cos(alpha(3)) sin(Q_3) * sin(alpha(3)) a(3) * cos(Q_3); sin(Q_3) cos(Q_3) * cos(alpha(3)) -cos(Q_3) * sin(alpha(3)) a(3) * sin(Q_3); 0 sin(alpha(3)) cos(alpha(3)) d(3); 0 0 0 1;];
    T_34 = [cos(Q_4) -sin(Q_4) * cos(alpha(4)) sin(Q_4) * sin(alpha(4)) a(4) * cos(Q_4); sin(Q_4) cos(Q_4) * cos(alpha(4)) -cos(Q_4) * sin(alpha(4)) a(4) * sin(Q_4); 0 sin(alpha(4)) cos(alpha(4)) d(4); 0 0 0 1;];
    finger_transform = T_01 * T_12 * T_23 * T_34;

end