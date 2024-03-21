function [theta_1, theta_2, d3, theta_4] = scara_ik(T,dh_parameters)
Ox= T(1,4)';   % Target Pose
Oy= T(2,4)';   % Target Pose
Oz= T(3,4)';   % Target Pose
nx = T(1,1);   % Target Orientation
ny = T(2,1);   % Target Orientation

a1 = dh_parameters(2,2);  % Dh Param
a2 = dh_parameters(3,2);  % Dh Param
d0 = dh_parameters(1,3);  % Dh Param
d3_3 = dh_parameters(4,3);  % Dh Param

Rad_to_Deg = 180/pi;
%--------------------------------------------------------------------------
tmp = (Ox^2 + Oy^2 - a1^2 - a2^2) / (2*a1*a2);

% Possible solution for theta_1 & theta_2
% theta_1(+) & theta_2(-)
% theta_1(-) & theta_2(+)
% theta_2 = atan2(-sqrt(1-tmp^2),tmp);
% theta_2 = atan2(sqrt(1-tmp^2),tmp); 
% theta_1 = atan2(Oy,Ox) + atan2(sqrt(Oy^2 + Ox^2 -(a2*cos(theta_2)+a1)^2),(a2*cos(theta_2)+a1));
% theta_1 = atan2(Oy,Ox) - atan2(sqrt(Oy^2 + Ox^2 -(a2*cos(theta_2)+a1)^2),(a2*cos(theta_2)+a1));


% if ((Ox < 0) && (Oy < 0)) || ((Ox >= 0) && (Oy < 0))
   
    theta_2 = atan2(-sqrt(1-tmp^2),tmp);
    theta_1 = atan2(Oy,Ox) + atan2(sqrt(Oy^2 + Ox^2 -(a2*cos(theta_2)+a1)^2),(a2*cos(theta_2)+a1));
    theta_4 =  -acos(cos(theta_1)*nx+sin(theta_1)*ny) - theta_2;

% elseif ((Ox >= 0) && (Oy >= 0)) || ((Ox < 0) && (Oy >= 0))
%     theta_2 = atan2(sqrt(1-tmp^2),tmp); 
%     theta_1 = atan2(Oy,Ox) - atan2(sqrt(Oy^2 + Ox^2 -(a2*cos(theta_2)+a1)^2),(a2*cos(theta_2)+a1));
%     theta_4 =  acos(cos(theta_1)*nx+sin(theta_1)*ny) - theta_2;
% end

theta_1 = theta_1 * Rad_to_Deg;
theta_2 = theta_2 * Rad_to_Deg;
d3 = Oz - d0 - d3_3;
theta_4 = theta_4 * Rad_to_Deg;




% Xe = 463.5;   % Target Pose 
% Ye = 0;       % Target Pose 
% Ze = 87;      % Target Pose 
% sigma = (pi/180)*0; % Target Orientation
% 
% XB = Xe - l4*cos(sigma);
% YB = Ye - l4*sin(sigma);
% d1 = Ze;
% cosT2 = (XB^2 + YB^2 - l2^2 - l3^2)/(2*l2*l3);
% sinT2 = sqrt(1 - cosT2^2);
% theta2 = acos(cosT2);
% theta1 = atan2(YB, XB) - atan2(l3*sinT2, (l2 + l3*cosT2));
% theta3 = sigma - theta1 - theta2;

end