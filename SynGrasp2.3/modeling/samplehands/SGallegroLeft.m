%    SGallegroLeft - The function builds the Allegro Hand (SimLab) model (left)
%
%    Usage: hand = SGallegroLeft
%
%    Returns:
%    hand = the AllegroHandLeft hand model
%
%    Contributors:
%    M. Malvezzi, V. Ruiz Garate, M. Pozzi
%
%    References:
%    1. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, A. Ajoudani. A Bio-Inspired 
%    Grasp Stiffness Control for Robotic Hands. Frontiers in Robotics 
%    and AI,2018. 
%    2. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, N. Tsagarakis, A.  
%    Ajoudani. Grasp Stiffness Control in Robotic Hands through Coordinated  
%    Optimization of Pose and Joint Stiffness. IEEE Robotics and Automation 
%    Letters, 3(4):3952-3959, October 2018.
%
%  Redistribution and use with or without
%  modification, are permitted provided that the following conditions are met:
%      * Redistributions of source code must retain he above copyright
%        notice, this list of conditions and the following disclaimer.
%      * Redistributions in binary form must reproduce the above copyright
%        notice, this list of conditions and the following disclaimer in the
%        documentation and/or other materials provided with the distribution.
%      * Neither the name of the <organization> nor the
%        names of its contributors may be used to endorse or promote products
%        derived from this software without specific prior written permission.
% 
%  THIS SOFTWARE IS PROVIDED BY M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo, ``AS IS'' AND ANY
%  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
%  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%  DISCLAIMED. IN NO EVENT SHALL M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES
%  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function newHand = SGallegroLeft(T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    1 - A L L E G R O   H A N D  D H  P A R A M E T E R S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

unit_length =1; % 1 for millimeters, unit_length=1000 for meters 
% remember to change it also in SGplotHand, where it is 1 by default

FinTipDim=28/unit_length; 

%%% Pre-allocation
DHpars{4} = [];
base{4} = [];
F{4} = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% index
% Data from drawing - unit: depends on unit_length
x11 = 45.098/unit_length;
y11 = 14.293/unit_length;

a12 = 54/unit_length;
a13 = 38.4/unit_length;
a14 = 43.7/unit_length;

% Reference frame and DH parameters
rotbaseindex = -5*pi/180;
Rindex = SGrotz(rotbaseindex)*SGrotz(-pi/2)*SGroty(-pi/2);

base{1} = [1 0 0 x11;
    0 1 0 y11;
    0 0 1 0;
    0 0 0 1];
base{1}(1:3,1:3) = Rindex;

DHpars{1}=[
    pi/2 0 0 0;
    0 a12 pi/2 0 ;            
    0 a13 0 0 ;
    0 a14 0 0];  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% middle
% Data from drawing - unit: depends on unit_length
x21 = 0;
y21 = 16.6/unit_length;

a22 = a12;
a23 = a13;
a24 = a14;

% Reference frame and DH parameters
Rmiddle = SGrotz(-pi/2)*SGroty(-pi/2);

base{2} = [1 0 0 x21;
    0 1 0 y21;
    0 0 1 0;
    0 0 0 1];
base{2}(1:3,1:3) = Rmiddle;

DHpars{2}=[
    pi/2 0 0 0;
    0 a22 pi/2 0 ;            
    0 a23 0 0 ;
    0 a24 0 0];  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% last
% Data from drawing - unit: depends on unit_length

x31 = -45.098/unit_length;
y31 = 14.293/unit_length;

a32 = a12;
a33 = a13;
a34 = a14;

% Reference frame and DH parameters
rotbaselast = 5*pi/180;
Rlast = SGrotz(rotbaselast)*SGrotz(-pi/2)*SGroty(-pi/2);

base{3} = [1 0 0 x31;
    0 1 0 y31;
    0 0 1 0;
    0 0 0 1];
base{3}(1:3,1:3) = Rlast;

DHpars{3}=[
    pi/2 0 0 0;
    0 a32 pi/2 0 ;            
    0 a33 0 0 ;
    0 a34 0 0];  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% thumb 
% Data from drawing - unit: depends on unit_length

rotthumb = -5*pi/180;

x41 = 16.958/unit_length;
y41 = -73.288/unit_length;
z41 = 18.2/unit_length;

x42 = 72.147/unit_length;
y42 = -78.116/unit_length;
z42 = 13.2/unit_length;

x43 = 72.147/unit_length;
y43 = -78.116/unit_length;
z43 = 13.2/unit_length;

x44 = 123.351/unit_length;
y44 = -82.596/unit_length;
z44 = 13.2/unit_length;


% Reference frame and DH parameters
base{4} = [1 0 0 x41;
    0 1 0 y41;
    0 0 1 z41;
    0 0 0 1];

base{4}(1:3,1:3) = SGrotz(rotthumb)*SGrotz(-pi/2)*SGroty(-pi/2);

d41 = -sqrt((z41-z42)^2+(y41-y42)^2);

a41 = z41-z42;

x42dh = x41 -d41*sin(rotthumb);
y42dh = y41 +d41*cos(rotthumb);

a42 = -sqrt((x42-x41)^2+(z42-z41)^2);
a43 = sqrt((x44-x43)^2+(y44-y43)^2);

a44 = 59.3/unit_length;

DHpars{4}=[ 
    pi/2 a41 0  d41 ;            
    pi/2 0 pi/2 a42 ;
    0 a43 -pi/2 0 ;
    0 a44 0 0];  

%%%%%%%%%%%%%%%%%%%%%
% build the hand...

for i = 1:length(DHpars)
    % number of joints for each finger
    joints = size(DHpars{i},1);
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
newHand.type = 'AllegroHandLeft';
if (nargin == 1)
    newHand.T=T;
end
