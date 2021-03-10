%% Task 1
clear all
clc

syms F1 F2 x1 x2 x3 x4 g l M J
g = 9.81;
eq1 = x2;
eq2 = ((F1+F2)/M)*cos(x3)+g;
eq3 = x4;
eq4 = ((F2-F1)/J)*l;

AlinSym = jacobian([eq1, eq2, eq3, eq4],[x1, x2, x3, x4]);
BlinSym = jacobian([eq1, eq2, eq3, eq4],[F1, F2]);

% With eq pts at x2 = 0, x3 = 0, x4 = 0 and M = 1kg, l = 1m, J = 1kgm^2
% x3 = 0 because drone is at hovering position so the angle of the drone
% would be 0
Alin = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
Blin = [0 0; 1 1; 0 0; -1 1];
Kalc = [Blin Alin*Blin Alin^2*Blin Alin^3*Blin];
rankKalc = rank(Kalc);

%% Task 2 part 1
% state space of drone
Ap = Alin;
Bp = Blin;
Cp = eye(4);
Dp = 0.*Cp*Bp;
sysP = ss(Ap, Bp, Cp, Dp);
% state feedback gains
K = [5 5.5 -5 -5.5; 5 5.5 5 5.5];
% closed loop response
Acl = Ap-Bp*K;
Bcl = eye(4);
Ccl = eye(4);
Dcl = eye(4);
sysCl = ss(Acl, Bcl, Ccl, Dcl);
time = 0:0.05:8;
init = [1;1;1;1];
x = initial(sysCl,init,time);
x1 = [1 0 0 0]*x';
x2 = [0 1 0 0]*x';
x3 = [0 0 1 0]*x';
x4 = [0 0 0 1]*x';
force = -K*x'+[g/2;g/2];
% settling time matrix
LCG = [time; x'];
% plotting linear responses
figure(1);
plot(time,x1,time,x2,time,x3,time,x4);
title('Response of linearized system')
xlabel('Time (sec)')
legend('x1','x2','x3','x4')
figure(2);
plot(time,force(1,:),time,force(2,:));
title('Force inputs of linearized system')
xlabel('Time (sec)')
legend('F1','F2');
% plotting nonlinear responses
[t,y] = ode45(@nonlinearSys,[0 10],[1; 1; 1; 1]);
NCG = [t'; y'];
figure(3);
plot(t,y(:,1),t,y(:,2),t,y(:,3),t,y(:,4));
title('Responses of nonlinear system')
xlabel('Time (sec)')
legend('x1','x2','x3','x4')
forceNL = -K*y'+[g/2;g/2];
figure(4);
plot(t,forceNL(1,:),t,forceNL(2,:));
title('Force inputs of nonlinear system')
xlabel('Time (sec)')
legend('F1','F2');

%% Task 2 part 2
ACLlin = Alin - Blin * K;
[clV, clD] = eig(Acl);

%% Task 3
%penalizing x3 and x4 10 times more than x1 and x2
Q = diag([1, 1, 8500, 70]);
R = [1 0; 0 1];
[Klqr,S,E]=lqr(Alin,Blin,Q,R);
LQRAcl = Alin - Blin*Klqr;
eigLQR = eig(LQRAcl);
% u = -K*x

%% Task 4
% plotting response with LQR K on linearized system
Acl = Ap-Bp*Klqr;
Bcl = eye(4);
Ccl = eye(4);
Dcl = eye(4);
sysCl = ss(Acl, Bcl, Ccl, Dcl);
time = 0:0.05:8;
init = [1;1;1;1];
x = initial(sysCl,init,time);
LOG = [time; x'];
x1 = [1 0 0 0]*x';
x2 = [0 1 0 0]*x';
x3 = [0 0 1 0]*x';
x4 = [0 0 0 1]*x';
force = -K*x'+[g/2;g/2];
figure(5)
plot(time,x1,time,x2,time,x3,time,x4);
title('Response of linearized system')
xlabel('Time (sec)')
legend('x1','x2','x3','x4')
figure(6)
plot(time,force(1,:),time,force(2,:));
title('Force inputs of linearized system')
xlabel('Time (sec)')
legend('F1','F2');
% plotting nonlinear responses
[t,y] = ode45(@LQRnonlin,[0 10],[1; 1; 1; 1]);
NOG = [t'; y'];
figure(7);
plot(t,y(:,1),t,y(:,2),t,y(:,3),t,y(:,4));
title('Responses of nonlinear system')
xlabel('Time (sec)')
legend('x1','x2','x3','x4')
forceNL = -K*y'+[g/2;g/2];
figure(8);
plot(t,forceNL(1,:),t,forceNL(2,:));
title('Force inputs of nonlinear system')
xlabel('Time (sec)')
legend('F1','F2');

