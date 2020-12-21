%    SGparadigmatic
%
%    The function builds the Paradigmatic Hand model
%
%    Usage: hand = SGparadigmatic
%
%    Returns:
%    hand = the Paradigmatic hand model
%
%    References:
%    M. Gabiccini, A. Bicchi, D. Prattichizzo, and M. Malvezzi, 
%    On the role of hand synergies in the optimal choice of grasping forces?
%    Autonomous Robots, pp. 1-18.
%
%    See also: SG3Fingered, SGDLRHandII, SGmodularHand

%    Copyright (c) 2012 M. Malvezzi, G. Gioioso, G. Salvietti, D.
%    Prattichizzo, A. Bicchi
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%    SynGrasp is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    SynGrasp is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with SynGrasp. If not, see <http://www.gnu.org/licenses/>.

function newHand = SGparadigmatic(T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    1 - P A R A D I G M A T I C   H A N D    P A  R A M E T E R S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

standardFix=rotz(90);
standardFix=rotm2tform(standardFix);

%%% Pre-allocation
DHpars{5} = [];
base{5} = [];
F{5} = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Thumb
DHpars{1}=[
    deg2rad(-90) 18	0	0;
deg2rad(65.41) 24.54 0 -2.18;
0 30	0	5.25;
0 35	0	0];

base{1} = [0.925416578398323,-0.163175911166535,-0.342020143325669,27.9000000000000;-0.318795777597168,-0.823172944645501,-0.469846310392954,-14.7400000000000;-0.204874128702862,0.543838142482326,-0.813797681349374,87.030000000000;0,0,0,1];
base{1} = standardFix*base{1};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Index
DHpars{2}=[
deg2rad(90) 18	0	0;
0 37	0	0;
0 25	0	0;
0 27	0	0];             % DIP joint (flexion/extention)
base{2} = [0.422618261740699,-0.906307787036650,0,43;0,0,-1,5.90000000000000;0.906307787036650,0.422618261740699,0,134.480000000000;0,0,0,1];
base{2} = standardFix*base{2};
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Middle
DHpars{3}=[
    deg2rad(90) 18	0	0;
0 44	0	0;
0 25	0	0;
0 27	0	0];        % DIP joint (flexion/extention)
base{3} = [0.0869434357387572,-0.993768017875764,0.0697564737441253,15;0.00607967728062676,-0.0694910293014737,-0.997564050259824,6.90000000000000;0.996194698091746,0.0871557427476582,0,141.480000000000;0,0,0,1];
base{3} = standardFix*base{3};
%%%%%%%%%%%%%%%%%%%%%%%%%%% Ring
DHpars{4}=[
    deg2rad(90) 18	0	0;
0 39	0	0;
0 25	0	0;
0 27	0	0];        % DIP joint (flexion/extention)

base{4} = [-0.0858316511774313,-0.981060262190407,0.173648177666930,-11.5000000000000;-0.0151344359013386,-0.172987393925089,-0.984807753012208,1.90000000000000;0.996194698091746,-0.0871557427476582,0,137.480000000000;0,0,0,1];
base{4} = standardFix*base{4};
%%%%%%%%%%%%%%%%%%%%%%%%%%% Little
DHpars{5}=[
    deg2rad(90) 18	0	0;
0 32	0	0;
0 25	0	0;
0 27	0	0];        % DIP joint (flexion/extention)

base{5} = [-0.250000000000000,-0.933012701892219,0.258819045102521,-35;-0.0669872981077807,-0.250000000000000,-0.965925826289068,-4.10000000000000;0.965925826289068,-0.258819045102521,0,123.480000000000;0,0,0,1];
base{5} = standardFix*base{5};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:length(DHpars)
    % number of joints for each finger
    joints = size(DHpars{i},2);
    % initialize joint variables
    q = zeros(joints,1);
    % make the finger
     if (nargin == 1)
        F{i} = SGmakeFinger(DHpars{i},T*base{i},q);
    else
        F{i} = SGmakeFinger(DHpars{i},base{i},q);
    end
end

newHand = SGmakeHand(F);
newHand.type = 'UBHand';

