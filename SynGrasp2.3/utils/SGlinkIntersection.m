%  This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D. Prattichizzo,
%  All rights reserved.
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

function alpha = SGlinkIntersection(seg,struct2,epsilon)

if(~SGisSeg(seg))
   error 'Argument seg should be a seg-structure' 
end
if (nargin<3)
    epsilon=1e-4;
    sprintf('default value of epsilon set to %d',epsilon);
end
switch struct2.type
    case 'cube'
        struct2_m = SGcube(struct2.Htr,struct2.dim(1)+3,struct2.dim(2)+3,struct2.dim(3)+3);
        alpha = SGintSegCube(seg,struct2_m);
    case 'cyl'
        struct2_m = SGcylinder(struct2.Htr,struct2.h,struct2.radius,struct2.res);
        alpha = SGintSegCyl(seg,struct2_m,epsilon);
    case 'sph'
        struct2_m = SGsphere(struct2.Htr,struct2.radius,struct2.res);
        alpha = SGintSegSph(seg,struct2_m,epsilon);
    otherwise
        error 'bad input arguments'
end
end