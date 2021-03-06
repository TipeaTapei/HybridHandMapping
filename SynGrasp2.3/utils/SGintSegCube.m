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

function alpha = SGintSegCube(seg,cube)

%splitting the cube into its faces,
%retrieving vertices:

%V=[cube.faces.f1,cube.faces.f2,cube.faces.f3,cube.faces.f4,cube.faces.f5,cube.faces.f6]; % F is a 3-by-24 matrix

V = [];

for i=1:6
    V = [V cube.faces.ver{i}];
end


Inv = inv(cube.Htr);
V_tilde = zeros(size(Inv,1),24);
for cont=1:24
    V_tilde(:,cont) = Inv*[V(:,cont);1];
end
%evaluating intersections: 
alphaVect = zeros(1,6);
for i=1:4:24
    CM = [V(:,i)' 1 ; V(:,i+1)' 1 ; V(:,i+2)' 1];
    if rank(CM) <= 2
        error('bad cube definition: vertices on face %d are aligned',SGindexReduction(i))
    end
    x1 = CM(1,1); 
    x2 = CM(2,1);
    x3 = CM(3,1);
    y1 = CM(1,2);
    y2 = CM(2,2);
    y3 = CM(3,2);
    z1 = CM(1,3);
    z2 = CM(2,3);
    z3 = CM(3,3);
    A = y1*(z2 - z3) - z1*(y2 - y3) + (y2*z3 - y3*z2);
    B = -(x1*(z2 - z3) - z1*(x2 - x3) + (x2*z3 - x3*z2));
    C = x1*(y2 - y3) - y1*(x2 - x3) + (x2*y3 - x3*y2);
    D = -(x1*(y2*z3 - y3*z2) - y1*(x2*z3 - x3*z2) + z1*(x2*y3 - x3*y2));
    DEN = (A*(seg.p1(1) - seg.p0(1)) + B*(seg.p1(2) - seg.p0(2)) + C*(seg.p1(3) - seg.p0(3)));
    if (DEN == 0)
        alphaVect(SGindexReduction(i)) = NaN;
        break
    end
    alphaTmp = -(A*seg.p0(1) + B*seg.p0(2) +C*seg.p0(3) + D) / DEN;
    %check alpha:
    j = SGindexReduction(i);
    
    if (alphaTmp <= 0.0 || norm(alphaTmp) > 1.0)
       alphaVect(j) = NaN;
    else
       alphaVect(j) = alphaTmp;
    end
end

for k=1:length(alphaVect)
    if ~isnan(alphaVect(k))
        alpha = alphaVect(k);
        X_tmp = seg.p0(1) + alpha*(seg.p1(1) - seg.p0(1));
        Y_tmp = seg.p0(2) + alpha*(seg.p1(2) - seg.p0(2));
        Z_tmp = seg.p0(3) + alpha*(seg.p1(3) - seg.p0(3));
        P_tilde = inv(cube.Htr)*[X_tmp;Y_tmp;Z_tmp;1];
        X_tmp = P_tilde(1);
        Y_tmp = P_tilde(2);
        Z_tmp = P_tilde(3);
        if ((X_tmp >= min(V_tilde(1,:))) && (X_tmp <= max(V_tilde(1,:))) && (Y_tmp >= min(V_tilde(2,:))) && (Y_tmp <= max(V_tilde(2,:))) && (Z_tmp >= min(V_tilde(3,:))) && (Z_tmp <= max(V_tilde(3,:))))
            alphaVect(k) = alpha;
        else
            alphaVect(k) = NaN;
        end
    else
        alphaVect(k) = NaN;
    end
end
alpha = min(alphaVect);
end