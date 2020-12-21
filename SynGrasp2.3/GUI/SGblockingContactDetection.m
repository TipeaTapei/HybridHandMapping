%  SGblockingContactDetection - function used by SGSyngrasp_GUI.m
% 
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

function [new_hand,new_object] = SGblockingContactDetection(hand,object,nq,new_q)

step = 0.01;
% on which finger is the joint and where is on the finger
finger = hand.qin(nq);

nqf = 0;
for i =1:nq

    if (hand.qin(i) == finger)
        
        nqf = nqf +1;
    end

end

% is there a contact point "after" the joint on that finger?
iscp = 0;

indexes = [];

if (size(hand.cp,2) > 0)
indexes = find(hand.cp(4,:) == finger);

for i=1:length(indexes)

    if(hand.cp(5,indexes(i)) >= nqf)
        iscp = 1;
    end
end
end
q_start = hand.q(nq);
q_final = new_q;

if(iscp)
    
    if((q_final >= q_start) || (nqf == 1)) %opening the finger or moving the abd/add joint
    
    new_object = object;
    new_hand = hand;
    
    else
        
    q = hand.q;
    q(nq) = new_q;

    hand = SGmoveHand(hand,q);
    
    for i=1:length(indexes)
    
        hand = SGremoveContact(hand,hand.cp(4,indexes(i)),hand.cp(5,indexes(i)),hand.cp(6,indexes(i)));
            
    end
    
    new_cp = SGcontactDetection(hand,object,finger);
    
    for i = 1:size(new_cp,1)
    
        hand = SGaddContact(hand,1,finger,new_cp(i,1),new_cp(i,2));
    end
        
    [new_hand,new_object] = SGcontact(hand,object);
    end
    
    
    
%no contact points    
else
    
    q = hand.q(nq);
    if(new_q == q)
       
        new_hand = hand;
        new_object = object;
       
    end
    
    if(new_q > q)
    while (q < new_q)
    
    q = q + step;
    q_tot = hand.q;
    q_tot(nq) = q;
    hand = SGmoveHand(hand,q_tot);
    new_cp = SGcontactDetection(hand,object,finger);
    tmp_cp = [];
    
    for i=1:size(new_cp,1)

        if(new_cp(i,1) >= nqf)
            tmp_cp = [tmp_cp; new_cp(i,:)];
        end
    end
    
    new_cp = tmp_cp;

    if (size(new_cp,1) == 1)
    hand = SGaddContact(hand,1,finger,new_cp(1,1),new_cp(1,2));
    [new_hand,new_object] = SGcontact(hand,object);
    break;
    else
        new_hand = hand;
        new_object = object;
        
    end
    
    end
    
    elseif(new_q < q)
    while (q > new_q)
    
    q = q - step;
    q_tot = hand.q;
    q_tot(nq) = q;
    hand = SGmoveHand(hand,q_tot);
    new_cp = SGcontactDetection(hand,object,finger);
    tmp_cp = [];
    
    for i=1:size(new_cp,1)

        if(new_cp(i,1) >= nqf)
            tmp_cp = [tmp_cp; new_cp(i,:)];
        end
    end
    
    new_cp = tmp_cp;

    if (size(new_cp,1) == 1)
    hand = SGaddContact(hand,1,finger,new_cp(1,1),new_cp(1,2));
    [new_hand,new_object] = SGcontact(hand,object);
    break;
    else
        new_hand = hand;
        new_object = object;
    end
    
    end
    end
    
    
    
%     q = hand.q;
%     q(nq) = new_q;
% 
%     hand = SGmoveHand(hand,q);
%     new_cp = SGcontactDetection(hand,object,finger);
%     
%     
% tmp_cp = [];
% for i=1:size(new_cp,1)
% 
%     if(new_cp(i,1) >= nqf)
%         tmp_cp = [tmp_cp; new_cp(i,:)];
%     end
% end
%     
% new_cp = tmp_cp;
% 
% if (size(new_cp,1) == 1)
% hand = SGaddContact(hand,1,finger,new_cp(1,1),new_cp(1,2));
% [new_hand,new_object] = SGcontact(hand,object);
% 
% else
%   
% %bad condition
%     if(size(new_cp,1) > 1 || (size(new_cp,1) == 1 && SGpointInSolid(hand.ftips(:,finger),object)))
%         while((size(new_cp,1) > 1) || (size(new_cp,1)==0))
%     
%         q_new = q_start + (q_final - q_start)/2;
%         q = hand.q;
%         q(nq) = q_new;
%         hand = SGmoveHand(hand,q);
%         new_cp = SGcontactDetection(hand,object,finger);
%     
%             if(size(new_cp,1) > 1)
%        
%             q_final = q_new;
%         
%             elseif (size(new_cp,1) == 0) 
%         
%             q_start = q_new;
%         
%             end
%     
%         end
% 
% hand = SGaddContact(hand,1,finger,new_cp(1,1),new_cp(1,2));
% [new_hand,new_object] = SGcontact(hand,object);
% new_object.G
%     else
%         
%         new_hand = hand;
%         new_object = object;
%         
%     end
% end
%     
    
    
end
end