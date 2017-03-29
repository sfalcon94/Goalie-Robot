%Input: x and y coordinates of ball received from vision server.
%Output: Actuator angles required to place arm end at (x,y) in quadrant 1
%for a 2-DoF arm.
%Known: Arm linkage lengths (l_1 and l_2).
function [phi_1, phi_2] = inverse_kinematics(U, V)
l_1=1;
l_2=1;

r = sqrt(U^2+V^2);
gamma = atan(V/U);
alpha = acos((-l_2^2 + l_1^2 + r^2)/(2*l_1*r));
beta = acos((-r^2+l_1^2+l_2^2)/(2*l_1*l_2));
phi_1 = gamma - alpha;
phi_2 = pi - beta;

if isnan(phi_1)
    phi_1 = 0;
end

if isnan(phi_2)
    phi_2 = 0;
end

end