function [ theta_1, theta_2 ] = arm_angles( X_ball, Y_ball )
%Input: x and y ball coordinates from vision server.
%Output: Actuator angles according to inverse_kinematics with signs
%corrected for quadrant.
%Known: Arm linkage lengths (l_1 and l_2).
V = abs(Y_ball);
U = abs(X_ball);
% Quadrant 1
if X_ball > 0 && Y_ball > 0
    [phi_1, phi_2]=inverse_kinematics(U,V);
    theta_1=phi_1;
    theta_2=phi_2;
    
% Quadrant 4
elseif X_ball > 0 && Y_ball < 0
    [phi_1, phi_2]=inverse_kinematics(U,V);
    theta_1=phi_1 - pi/2;
    theta_2=phi_2;
    
% Quadrant 3
elseif X_ball < 0 && Y_ball < 0
    [phi_1, phi_2]=inverse_kinematics(U,V);
    theta_1=phi_1 - pi;
    theta_2=phi_2;
    
% Quadrant 2
else
    [phi_1, phi_2]=inverse_kinematics(U,V);
    theta_1=phi_1 + pi/2;
    theta_2=phi_2;
end

end

