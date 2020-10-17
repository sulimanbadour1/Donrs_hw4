function q = IK_ROBOT_POINTS(point)
%IK_ROBOT_POINTS Explination is here for IK parameters 
%   Elbow up configuration
%% Constant parameters
L1 = 1;  
L2 = 1;
L3 = 1;
x = point(1);
y = point(2);
% z = L1 - ( sin(q2+q3) + sin(q2) )
z = point(3);

% For getting q2 and q3
X = sqrt( x^2 + y^2); 
Y = L1 - z;
%% extract q1
q1 = atan2(y,x);

%% extract q2 $ q3

q3 = acos( ( X^2 + Y^2 - L2^2 - L3^2)/ (2 * L2 * L3) );

q2 = - atan2( L3 * sin(q3), (L2 + L3 * cos(q3))) + atan2(Y,X);

q = [q1 q2 q3];


end