%    SGplotPalm_allegro - Plot the palm of the Allegro Hand model
%
%    The function plots the palm of the hand model given as argument
%
%    Usage: SGplotPalm_allegro
%
%    Arguments:
%    hand = the hand whose palm the user wants to plot
%   
%    See also: SGplotHand
%
%    Contributors:
%    V. Ruiz Garate, M. Pozzi
%
%    References:
%    1. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, Arash A.. A Bio-Inspired 
%    Grasp Stiffness Control for Robotic Hands. Frontiers in Robotics 
%    and AI,2018. 
%    2. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, N. Tsagarakis, A.  
%    Ajoudani. Grasp Stiffness Control in Robotic Hands through Coordinated  
%    Optimization of Pose and Joint Stiffness. IEEE Robotics and Automation 
%    Letters, 3(4):3952-3959, October 2018.
% 
%  Redistribution and use with or without
%  modification, are permitted provided that the following conditions are met:
%      * Redistributions of source code must retain the above copyright
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


function SGplotPalm_allegro(hand,radius,nps)

if nargin == 1
    % default settings
    radius = 5/1000;
    nps = 5;
elseif nargin == 2
    nps = 5;
end

F = hand.F;
n = hand.n;
basepoints = zeros(3,n+1);

for i= 1:length(F)
    point = [hand.F{i}.base(1,4)
        hand.F{i}.base(2,4)
        hand.F{i}.base(3,4)];
    basepoints(:,i)= point;
end



c = length(F);
if abs(hand.F{2}.base(1,4)-hand.F{c}.base(1,4)) > abs(hand.F{2}.base(2,4)-hand.F{c}.base(2,4))
    
    puntodx = [hand.F{c}.base(1,4)
        hand.F{1}.base(2,4)
        hand.F{c}.base(3,4)] ;
    
else
    puntodx = [hand.F{1}.base(1,4)
        hand.F{c}.base(2,4)
        hand.F{c}.base(3,4)] ;
end


basepoints(:,n+1) = puntodx;

cp = basepoints';

co = mean(cp);


[nc,tmp] = size(cp);


for i = 1:nc
    ncp(i,:) = (co - cp(i,:))/norm(co - cp(i,:));
end



X = cp;

for i = 1:nc
    [filletx,fillety,filletz] = sphere(nps);
    spherecenter = cp(i,:) + radius*ncp(i,:);
    for j = 1:nps+1
        for k = 1:nps+1
            fillett = radius*[filletx(j,k) fillety(j,k) filletz(j,k)]+spherecenter;
            X = [X;fillett];
        end
    end
end

k = convhulln(X);
trisurf(k,X(:,1),X(:,2),X(:,3));
colormap([0.3 0.3 0.3]);
hold on

