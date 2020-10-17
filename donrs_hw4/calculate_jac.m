function J = calculate_jac(q)
%calculate the jacobian for this robot RRR
% Details
%% CONSTANT PARAMETERS
L1 = 1 ;
L2 = 1 ;
L3 = 1 ;
q1 = q(1);
q2 = q(2);
q3 = q(3);
%% FK
H = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q2) * Ty(L3);
%% Jacobian calculation
R = H(1:3,1:3);
R_inv = [R' zeros(3,1);0 0 0 1 ];
%1st joint
Td= Rzd(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3)*R_inv;
J1 = [Td(1,4), Td(2,4), Td(3,4)]';% Td(3,2), Td(1,3), Td(2,1)]' ;

%2nd joint 
Td=Rz(q1)*Tz(L1)*Ryd(q2)*Tx(L2)*Ry(q3)*Tx(L3)*R_inv;
J2 = [Td(1,4), Td(2,4), Td(3,4)]';% Td(3,2), Td(1,3), Td(2,1)]';

%3rd joint
Td=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ryd(q3)*Tx(L3)*R_inv;
J3 = [Td(1,4), Td(2,4), Td(3,4)]';%, Td(3,2), Td(1,3), Td(2,1)]' ;

%full matrix of jacobians
J = [ J1 J2 J3 ];
end