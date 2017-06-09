 function [Output,sol,SolApprox] = PGIVince(t_target,Q_target,ThetaEstimate)
 %#codegen
 
 %Output=[[0 90 360 180 -180 0]';ThetaEncoders(7:9)];
 ThetaEstimate(2) = -ThetaEstimate(2);
 JointOutput=ThetaEstimate(1:6)/180*pi;
%Testing Purposes ---------------------------------------
 % Paramètres physiques du robot Jaco
    D1 = 0.2755;
    D2 = 0.41; 
    D3 = 0.2073; 
    D4 = 0.0743;
    D5 = 0.0743;
    D6 = 0.1687;
    E2 = 0.0098;
    aa = 11*pi/72;

% Paramètres Intermédiaires
    sa = sin(aa);
    s2a = sin(2*aa);
    d4b = D3 + (sa/s2a)*D4;
    d5b = sa/s2a*D4 + (sa/s2a)*D5;
    d6b = sa/s2a*D5 + D6;

% Paramètres DH (Kinova) 
    a=    [ 0,      D2,  0,      0,      0,      0];
    d=    [ D1,     0,   -E2,    -d4b,   -d5b,   -d6b];
    alpha=[ pi/2,   pi,  pi/2,   2*aa,   2*aa,   pi];
    sample_time = 0.005;
%Testing Purposes ---------------------------------------

%XXXXXXXXXXXXXXXXXXXXXX PGI XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX       
%--- PARAMETRES DE LA FONCTION -------------------------------------------------------------------------------------
    iter_max = 20;
    erreur = 0;

    %--- INITIALISATION DE LA FONCTION A MINIMISER ---------------------------------------------------------------------
    f = zeros(12,1);
    
%--- BOUCLE PRINCIPALE ---------------------------------------------------------------------------------------------

for iter = 1:iter_max  
    %--- CALCUL DE LA JACOBIENNE -----------------------------------------------------------------------------------
    theta1=JointOutput(1);
    theta2=JointOutput(2);
    theta3=JointOutput(3);
    theta4=JointOutput(4);
    theta5=JointOutput(5);
    theta6=JointOutput(6);
    
    a1=[a(1)*cos(theta1);a(1)*sin(theta1);d(1)];
    a2=[a(2)*cos(theta2);a(2)*sin(theta2);d(2)];
    a3=[a(3)*cos(theta3);a(3)*sin(theta3);d(3)];
    a4=[a(4)*cos(theta4);a(4)*sin(theta4);d(4)];
    a5=[a(5)*cos(theta5);a(5)*sin(theta5);d(5)];
    a6=[a(6)*cos(theta6);a(6)*sin(theta6);d(6)];
    
    Q1=[cos(theta1) -cos(alpha(1))*sin(theta1) sin(alpha(1))*sin(theta1);
        sin(theta1) cos(alpha(1))*cos(theta1)  -sin(alpha(1))*cos(theta1);
        0 sin(alpha(1)) cos(alpha(1))];
    Q2=[cos(theta2) -cos(alpha(2))*sin(theta2) sin(alpha(2))*sin(theta2);
        sin(theta2) cos(alpha(2))*cos(theta2)  -sin(alpha(2))*cos(theta2);
        0 sin(alpha(2)) cos(alpha(2))];
    Q3=[cos(theta3) -cos(alpha(3))*sin(theta3) sin(alpha(3))*sin(theta3);
        sin(theta3) cos(alpha(3))*cos(theta3)  -sin(alpha(3))*cos(theta3);
        0 sin(alpha(3)) cos(alpha(3))];
    Q4=[cos(theta4) -cos(alpha(4))*sin(theta4) sin(alpha(4))*sin(theta4);
        sin(theta4) cos(alpha(4))*cos(theta4)  -sin(alpha(4))*cos(theta4);
        0 sin(alpha(4)) cos(alpha(4))];
    Q5=[cos(theta5) -cos(alpha(5))*sin(theta5) sin(alpha(5))*sin(theta5);
        sin(theta5) cos(alpha(5))*cos(theta5)  -sin(alpha(5))*cos(theta5);
        0 sin(alpha(5)) cos(alpha(5))];
    Q6=[cos(theta6) -cos(alpha(6))*sin(theta6) sin(alpha(6))*sin(theta6);
        sin(theta6) cos(alpha(6))*cos(theta6)  -sin(alpha(6))*cos(theta6);
        0 sin(alpha(6)) cos(alpha(6))];
    
    P1 =    Q1;
    P2 = P1*Q2;
    P3 = P2*Q3;
    P4 = P3*Q4;
    P5 = P4*Q5;
    P6 = P5*Q6;
    Q=P6;
    
    
    e1 = [0 0 1]';
    e2 = P1*e1;
    e3 = P2*e1;
    e4 = P3*e1;
    e5 = P4*e1;
    e6 = P5*e1;
    r1 = a1+P1*a2+P2*a3+P3*a4+P4*a5+P5*a6;
    r2 = P1*a2+P2*a3+P3*a4+P4*a5+P5*a6;
    r3 = P2*a3+P3*a4+P4*a5+P5*a6;
    r4 = P3*a4+P4*a5+P5*a6;
    r5 = P4*a5+P5*a6;
    r6 = P5*a6;
    
    E = [0 -1 0;1 0 0;0 0 0];
    s1 = [1,0,0]';
    s2 = [0,1,0]';
    s3 = [0,0,1]';
    EQ1 = E*Q;
    EQ2 = P1*E*Q2*Q3*Q4*Q5*Q6;
    EQ3 = P2*E*Q3*Q4*Q5*Q6;
    EQ4 = P3*E*Q4*Q5*Q6;
    EQ5 = P4*E*Q5*Q6;
    EQ6 = P5*E*Q6;
    
    R1 = [EQ1*s1, EQ2*s1, EQ3*s1, EQ4*s1, EQ5*s1, EQ6*s1];
    R2 = [EQ1*s2, EQ2*s2, EQ3*s2, EQ4*s2, EQ5*s2, EQ6*s2];
    R3 = [EQ1*s3, EQ2*s3, EQ3*s3, EQ4*s3, EQ5*s3, EQ6*s3];
    
    
    % Matrice Jacobienne
    
    J = [R1; R2; R3; ...
        cross(e1,r1), cross(e2,r2), cross(e3,r3), ...
        cross(e4,r4), cross(e5,r5), cross(e6,r6)];
    %     J(:,1)=-J(:,1);
    
    %conditionnement = cond(J);
    
    %--- PERFORMANCE ACTUELLE --------------------------------------------------------------------------------------
    % Orientation
    f(1:3) = (Q - Q_target)*s1;
    f(4:6) = (Q - Q_target)*s2;
    f(7:9) = (Q - Q_target)*s3;
    
    % Position
    f(10:12) = (r1 - t_target(1:3));
    
    %--- CRITERE D'ARRET -------------------------------------------------------------------------------------------
    Weight = [1 1 1 1 1 1 1 1 1 1 1 1]';
%     epsilon = 0.0000001^2;
      
%  arret = (f.*Weight)'*(f.*Weight) < epsilon;
    epsilon = 0.001;
    arret = max(abs(f)) < epsilon;
    
%     if isreal(JointOutput(1:6)) 
%     Output=[JointOutput(1:6)/pi*180;[0;0;0]];
%     else
%     Output=[JointTarget(1:6)/pi*180;[0;0;0]];    
%     end
	if arret       
        sol=1;
        Output = [JointOutput(1:6)/pi*180];
	%break;
    else
        sol=0;
        Output=ThetaEstimate;
	end
    %--- AJUSTEMENT POUR LA PROCHAINE ITERATION --------------------------------------------------------------------
        %dtheta = pinv(J) * f;  
        %     %Jacobienne amortie pour les singularites version Pascal Labrecque
       Ek = 0.5*(f')*f;
       Wn = Ek*eye(6) + 0.001*eye(6);
       Hk = (J')*J + Wn;
       Jinv = Hk\(J');
       dtheta = Jinv*f;
      JointOutput = JointOutput - dtheta;
      SolApprox = [JointOutput(1:6)/pi*180];
end
Output(2) = -Output(2);
SolApprox(2) = -SolApprox(2);
end  