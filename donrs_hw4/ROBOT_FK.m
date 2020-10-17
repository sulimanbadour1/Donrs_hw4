function H = ROBOT_FK(q)
%ROBOT_FK Explination is here 
%% Constant parameters 
L1 = 1;
L2 = 1;
L3 = 1;
q1 = q(1);
q2 = q(2);
q3 = q(3);
%% Forward kinematics
H = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3);
end

