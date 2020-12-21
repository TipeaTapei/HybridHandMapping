%The following code defines an hybrid joint-cartesian mapping between two
%anthropomorphic robotic hands as presented in:
% "Exploiting In-Hand Knowledge in Hybrid Joint-Cartesian Mapping
% for Intuitive Teleoperation of Anthropomorphic Robotic Hands"

%REQUIREMENTS
%In order to properly run, the software requires the characterization of the
%master and slave robotic hands and the definition of some parameters
%as defined in the following file
%Add the whole package to matlab workspace through HMinstall.m
%Syngrasp toolbox for matlab required
%

%SYSTEM SETUP
%This file evaluates the workspace variables necessary to define the hybrid
%mapping. 

%ROBOTIC HANDS STRUCTURE DEFINITION
%The software exploits Syngrasp functionalities for the definition of the
%robotic hands. Please define a Syngrasp implementation of the master and
%slave hand. Default values are set to reference paper experimentation
masterHand=SGparadigmaticHybrid(eye(4));
slaveHand=SGUBHandHybrid(eye(4));

%ROBOTIC HANDS DIRECT KINEMATIC FUNCTION
%The software requires the forward kinematic (fk) implementation for all the
%fingers of the master and slave hand. The function can be provided for the
%whole hand (1) or singularly for each finger (0)
%Thumb,Index,Middle,Ring,Pinkie

%set unique or separate fk for master
m_complete_fk=0;

%set unique or separate fk for slave
s_complete_fk=1;

%master hand forward kinematic functions. Each function should receive as
%input a row vector containing the 20 ordered joint values for all the hand
%fingers according to thumb-index-middle-ring-pinkie order. It should
%provide as output an homogeneous matrix describing each finger fingertip
%reference frame w.r.t the finger base frame.
%Change the value after "@" with the slave hand kinematic function/s
if m_complete_fk==1
    master_fk=@fkineHand;
else
    masterThumb_fk=@ParaFingerFkThumb;
    masterIndex_fk=@ParaFingerFkIndex;
    masterMiddle_fk=@ParaFingerFkMiddle;
    masterRing_fk=@ParaFingerFkRing;
    masterPinkie_fk=@ParaFingerFkPinkie;
end

%slave hand forward kinematic functions. Each function should receive as
%input a row vector containing the 20 ordered joint values for all the hand
%fingers according to thumb-index-middle-ring-pinkie order. It should
%provide as output an homogeneous matrix describing each finger fingertip
%reference frame w.r.t the finger base frame.
%Change the value after "@" with the slave hand kinematic function
if s_complete_fk==1
    slave_fk=@fkineHand;
else
    slaveThumb_fk=@ParaFingerThumbFk;
    slaveIndex_fk=@ParaFingerIndexFk;
    slaveMiddle_fk=@ParaFingerMiddleFk;
    slaveRing_fk=@ParaFingerRingFk;
    slavePinkie_fk=@ParaFingerPinkieFk;
end

%ROBOTIC HANDS INVERSE KINEMATIC FUNCTION
%The inverse kinematics is required only for the slave hand
%Change the value after "@" with the slave hand kinematic function
slaveThumb_ik=@thumbIkSlave;
slaveIndex_ik=@indexIkSlave;
slaveMiddle_ik=@middleIkSlave;
slaveRing_ik=@ringIkSlave;
slavePinkie_ik=@pinkieIkSlave;

%MASTER SLAVE JOINT MAPPING
%the joint mapping between master and slave dependes on the kinematics of
%the hands. The functions should be set accordingly
jThumbMapping=@jointThumbMapping
jIndexMapping=@jointIndexMapping
jMiddleMapping=@jointMiddleMapping
jRingMapping=@jointRingMapping
jPinkieMapping=@jointPinkieMapping

%ROBOTIC HAND JOINT LIMITS
%the limits are defined in the specific hand files
MasterHandParams
SlaveHandParams

%ROBOTIC HAND SPACE DISCRETIZATION
%It is necessary to define a discretization of the space enveloping the
%hand. The transformation between the origin of the chosen discretization
%and the hand origin must be defined in the master hand and slave hand
%transformation function:
%- Finger2Global maps from master hand space to the discretization
%- Global2Finger maps from discretization to master hand space
%- Finger2SlaveGlobal maps from slave hand space to the discretization
%- Global2SlaveFinger maps from discretization to slave hand space
%
%Master hand Global discretization dimensions(mm)
globalDim = [100,300,300];
%Slave hand Global discretization dimensions(mm)
globalSlaveDim = [300,350,300];

%Convex hull transition area expansion (see paper for info)
shapeFactor=1.2

%WORKSPACE EVALUATION
%The workspace evaluation will evaluate the configuration space of all the
%master and slave fingers and their control hulls
%Since the evaluation can requires some time, an already evaluated
%workspace can be loaded from 'workspace_ready.mat'
%To enable the evaluation set the variable to 1
enable_evaluation=0;

if enable_evaluation
    %MASTER HAND CONTROL HULL EVALUATION
    MasterHullEvaluation

    %SLAVE HAND CONTROL HULL EVALUATION
    SlaveHullEvaluation

    %MAP MASTER HULLS INTO SLAVE HULLS
    ControlHullMapping
end