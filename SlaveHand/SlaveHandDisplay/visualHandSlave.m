function visualHandSlave(hand,joints)

joints = joints';
joints = joints(:);

hand = SGmoveHand(hand,joints);
SGplotHand(hand)

end