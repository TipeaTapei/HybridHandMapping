%    References:

%    R. Meattini, D. Chiaravalli, G. Palli and C. Melchiorri - University of Bologna - Dept. of Electrical, Electronic and System Eng. (DEI) "Exploiting In-Hand Knowledge in Hybrid Joint-Cartesian Mapping for Intuitive Teleoperation of Anthropomorphic Robotic Hands." [Submitted for IEEE Robotics and Automation Letters (RA-L) with option for IEEE-RAS International Conference on Humanoid Robots (HUMANOIDS2020)]

%

%    Copyright (c) 2020. R. Meattini, D. Chiaravalli, G. Palli and C. Melchiorri

%

%    *Necessary disclaimer*: The files of this package are distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

This package contains the hybrid master to slave mapping presented in "Exploiting In-Hand Knowledge in Hybrid Joint-Cartesian Mapping for Intuitive Teleoperation of Anthropomorphic Robotic Hands".

Run HMinstall.m to add the package to matlab path.

Run HybridMapping.m to see the result of the proposed mapping and the related motion between the master and slave hans. It is also possible to evaluate the cartesian trajectory for the slave hand according to the proposed mapping starting from a vector of joint reference for all the fingers.

Run SystemSetup.m to evaluate the workspace of master and slave hand and generate the data structures to implement the hybrid mapping. A precompiled workspace can be loaded from workspace_ready.mat.

