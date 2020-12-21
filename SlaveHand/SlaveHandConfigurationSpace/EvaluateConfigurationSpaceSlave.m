
theta_step_t=0.05;
theta_step=0.05;
GlobalSlaveSpaceThumb=zeros(globalSlaveDim(1),globalSlaveDim(2),globalSlaveDim(3));
GlobalSlaveSpaceIndex=zeros(globalSlaveDim(1),globalSlaveDim(2),globalSlaveDim(3));
GlobalSlaveSpaceMiddle=zeros(globalSlaveDim(1),globalSlaveDim(2),globalSlaveDim(3));
GlobalSlaveSpaceRing=zeros(globalSlaveDim(1),globalSlaveDim(2),globalSlaveDim(3));
GlobalSlaveSpacePinkie=zeros(globalSlaveDim(1),globalSlaveDim(2),globalSlaveDim(3));

    
for i=-deg2rad(10):theta_step_t:deg2rad(10)
    for j=0:theta_step:deg2rad(85)
        for k=0:theta_step:deg2rad(90)
            for t=0:theta_step:deg2rad(70)
                thumb_i=i*160/20;
                theta=[j thumb_i k t,i j k t,i j k t,i j k t,i j k t];
                [T_be,T_0e]=slave_fk(theta);
                %thumb
                x=-T_be{1,1}(2,4);
                y=T_be{1,1}(1,4);
                z=T_be{1,1}(3,4);
                finger=[x,y,z];
                result=Finger2SlaveGlobal(finger);
                GlobalSlaveSpaceThumb(result(1),result(2),result(3))=1;
                %index
                x=-T_be{1,2}(2,4);
                y=T_be{1,2}(1,4);
                z=T_be{1,2}(3,4);
                finger=[x,y,z];
                result=Finger2SlaveGlobal(finger);
                GlobalSlaveSpaceIndex(result(1),result(2),result(3))=1;
                %index
                x=-T_be{1,3}(2,4);
                y=T_be{1,3}(1,4);
                z=T_be{1,3}(3,4);
                finger=[x,y,z];
                result=Finger2SlaveGlobal(finger);
                GlobalSlaveSpaceMiddle(result(1),result(2),result(3))=1;
                %ring
                x=-T_be{1,4}(2,4);
                y=T_be{1,4}(1,4);
                z=T_be{1,4}(3,4);
                finger=[x,y,z];
                result=Finger2SlaveGlobal(finger);
                GlobalSlaveSpaceRing(result(1),result(2),result(3))=1;
                %pinkie
                x=-T_be{1,5}(2,4);
                y=T_be{1,5}(1,4);
                z=T_be{1,5}(3,4);
                finger=[x,y,z];
                result=Finger2SlaveGlobal(finger);
                GlobalSlaveSpacePinkie(result(1),result(2),result(3))=1;   
            end
        end
    end
end


