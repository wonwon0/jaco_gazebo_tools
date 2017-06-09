function [answer] = PGI1(p_cible,Q_cible,X0)

% Parametres DH

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

%#codegen
% Calcul les positions articulaires correspondantes a la position cartesienne fournie
%
% Input
%	- theta_actu		--> Valeur initiale des positions articulaires (6x1)
%	- p_cible			--> Vecteur de la position cible (3x1)
%	- Q_cible			--> Matrice de rotation cible (3x3)
%
% Output
%	- theta_cible		--> Positions articulaires necessaires pour atteindre la cible
%	- erreur			--> Code de sortie (0 indique le succes)

theta_actu = X0(1:6);


%--- PARAMETRES DE LA FONCTION -------------------------------------------------------------------------------------
iter_max = 100;
epsilon = 0.0001^2;
erreur = 0;

%--- PARAMETRES DU ROBOT -------------------------------------------------------------------------------------------
theta_off	= -[0;90;-90;0;180;-100]*pi/180;
theta_actu	= theta_actu * pi/180;
theta		= theta_actu + theta_off;
theta(1)	= -theta(1);
theta_in = theta;

%--- INITIALISATION DE LA FONCTION A MINIMISER ---------------------------------------------------------------------
f = zeros(12,1);


% if abs(p_cible(1)-0.4471) > sqrt(epsilon)
%     test=1;
% end

%--- BOUCLE PRINCIPALE ---------------------------------------------------------------------------------------------

    %--- CALCUL DE LA JACOBIENNE -----------------------------------------------------------------------------------
    theta1=theta(1);
    theta2=theta(2);
    theta3=theta(3);
    theta4=theta(4);
    theta5=theta(5);
    theta6=theta(6);
    
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
        cross(e4,r4), cross(e5,r5), cross(e6,r6)]
    %     J(:,1)=-J(:,1);
    
    %conditionnement = cond(J);
    
    %--- PERFORMANCE ACTUELLE --------------------------------------------------------------------------------------
    % Orientation
    f(1:3) = (Q - Q_cible)*s1;
    f(4:6) = (Q - Q_cible)*s2;
    f(7:9) = (Q - Q_cible)*s3;
    
    % Position
    f(10:12) = r1 - p_cible(1:3);
    
    %--- CRITERE D'ARRET -------------------------------------------------------------------------------------------
    arret = f'*f < epsilon;
    if arret
%         theta_cible = (theta - theta_off) * 180/pi;
%         theta_cible(1) = -theta_cible(1);
%         erreur = 0;
        break;
    end
    
    
    
    %--- AJUSTEMENT POUR LA PROCHAINE ITERATION --------------------------------------------------------------------
    %dtheta = pinv(J) * f;
    
    %     %Jacobienne amortie pour les singularites version Pascal Labrecque
    Ek = 0.5*(f')*f;
    Wn = Ek*eye(6) + 0.001*eye(6);
    Hk = (J')*J + Wn;
    Jinv = Hk\(J');
    dtheta = Jinv*f;
    
    theta = theta - dtheta;

iter
         theta_cible = (theta - theta_off) * 180/pi;
         theta_cible(1) = -theta_cible(1);
     answer=theta_cible;
end























