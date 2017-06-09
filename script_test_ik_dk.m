
theta = [0 90 0 0 0 0]';
% theta_off	= -[0;90;-90;0;180;-100]*pi/180;
% theta = theta - theta_off;
% for i = 1:100
%     [pose,q] = PGDVince(theta);
%     pose
%     old_pose = pose;
%     [theta, sol, solaprox] = PGIVince(pose,q,theta);
%     theta
%     [pose,q] = PGDVince(theta);
%     pose
% end
theta = [0 10 0 0 0 0]';
[pose,q] = PGDVince(theta);
pose
speed = [0.01,0,0]';
dt = 0.01;
for i = 1:10
[theta, sol, solaprox] = PGIVince(pose + speed*dt,q,theta);
[pose,q] = PGDVince(theta);
(pose - old_pose)./dt
old_pose = pose;
end
for i = 1:10
[theta,sol,solaprox] = PGIVince(pose - speed*dt,q,theta);
[pose,q] = PGDVince(theta);
(pose - old_pose)./dt
old_pose = pose;
end