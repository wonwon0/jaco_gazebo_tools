 function [t_real,Q_real] = PGDVince(ThetaEncoders)

%Testing Purposes ---------------------------------------
 %ThetaEncoders = ThetaEncoders + [180;270;90;180;180;350];
%  ThetaEncoders = ThetaEncoders + -[0;90;-90;-360;-180;-180];
    ThetaEncoders(2) = -ThetaEncoders(2);
    %ThetaEncoders(3) = -ThetaEncoders(3);
    
 % Parametres physiques du robot Jaco
    D1 = 0.2755;
    D2 = 0.41; 
    D3 = 0.2073; 
    D4 = 0.0743;
    D5 = 0.0743;
    D6 = 0.1687;
    E2 = 0.0098;
    aa = 11*pi/72;

% Parametres Intermediaires
    sa = sin(aa);
    s2a = sin(2*aa);
    d4b = D3 + (sa/s2a)*D4;
    d5b = sa/s2a*D4 + (sa/s2a)*D5;
    d6b = sa/s2a*D5 + D6;

% Parametres DH (Kinova) 
    a=    [ 0,      D2,  0,      0,      0,      0];
    d=    [ D1,     0,   -E2,    -d4b,   -d5b,   -d6b];
    alpha=[ pi/2,   pi,  pi/2,   2*aa,   2*aa,   pi];
    %sample_time = 0.005;
%Testing Purposes ---------------------------------------

%XXXXXXXXXXXXXXXXXXXXXX PGI XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX       
    %--- CALCUL DE LA JACOBIENNE -----------------------------------------------------------------------------------
    theta1=ThetaEncoders(1)/180*pi;
    theta2=ThetaEncoders(2)/180*pi;
    theta3=ThetaEncoders(3)/180*pi;
    theta4=ThetaEncoders(4)/180*pi;
    theta5=ThetaEncoders(5)/180*pi;
    theta6=ThetaEncoders(6)/180*pi;
    
    a1=[a(1)*cos(theta1);a(1)*sin(theta1);d(1)];
    a2=[a(2)*cos(theta2);a(2)*sin(theta2);d(2)];
    a3=[a(3)*cos(theta3);a(3)*sin(theta3);d(3)];
    a4=[a(4)*cos(theta4);a(4)*sin(theta4);d(4)];
    a5=[a(5)*cos(theta5);a(5)*sin(theta5);d(5)];
    a6=[a(6)*cos(theta6);a(6)*sin(theta6);d(6)];
    
    Q1=[cos(theta1) -cos(alpha(1))*sin(theta1)   sin(alpha(1))*sin(theta1);
        sin(theta1)  cos(alpha(1))*cos(theta1)  -sin(alpha(1))*cos(theta1);
        0            sin(alpha(1))               cos(alpha(1))];
    
    Q2=[cos(theta2) -cos(alpha(2))*sin(theta2)   sin(alpha(2))*sin(theta2);
        sin(theta2)  cos(alpha(2))*cos(theta2)  -sin(alpha(2))*cos(theta2);
        0            sin(alpha(2))               cos(alpha(2))];
    
    Q3=[cos(theta3) -cos(alpha(3))*sin(theta3)   sin(alpha(3))*sin(theta3);
        sin(theta3)  cos(alpha(3))*cos(theta3)  -sin(alpha(3))*cos(theta3);
        0            sin(alpha(3))               cos(alpha(3))];
    
    Q4=[cos(theta4) -cos(alpha(4))*sin(theta4)   sin(alpha(4))*sin(theta4);
        sin(theta4)  cos(alpha(4))*cos(theta4)  -sin(alpha(4))*cos(theta4);
        0            sin(alpha(4))               cos(alpha(4))];
    
    Q5=[cos(theta5) -cos(alpha(5))*sin(theta5)   sin(alpha(5))*sin(theta5);
        sin(theta5)  cos(alpha(5))*cos(theta5)  -sin(alpha(5))*cos(theta5);
        0            sin(alpha(5))               cos(alpha(5))];
    
    Q6=[cos(theta6) -cos(alpha(6))*sin(theta6)   sin(alpha(6))*sin(theta6);
        sin(theta6)  cos(alpha(6))*cos(theta6)  -sin(alpha(6))*cos(theta6);
        0            sin(alpha(6))               cos(alpha(6))];
    
    P1 =    Q1;
    P2 = P1*Q2;
    P3 = P2*Q3;
    P4 = P3*Q4;
    P5 = P4*Q5;
    Q_real = P5*Q6;
%     x=Q_real(:,1)/norm(Q_real(:,1));
%     y=cross(Q_real(:,3),x);
%     y=y/norm(y);
%     z=cross(x,y);
%     Q_real = [x y z]
 
    t_real = a1+P1*a2+P2*a3+P3*a4+P4*a5+P5*a6;

end        



        
