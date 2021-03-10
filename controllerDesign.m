clear all
clc

% State space model for roll dynamics
Ap = [0 1; 0 0];
Bp = [0; 1];
Cp = [1 0];
Dp = 0.*Cp*Bp;
sysP = ss(Ap, Bp, Cp, Dp);
% Is system stabilizable and detectable
eigA = eig(Ap);
% stabilizability
stable = [eigA(1).*eye(2)-Ap, Bp];
rank(stable)
% full row rank so stabilizable
% detectability
detect = [eigA(1).*eye(2)-Ap; Cp];
rank(detect')
% full column rank so detectable

% Checking controllability
KalC = [Bp Ap*Bp];
rankC = rank(KalC);
% Checking observability
KalO = [Cp; Cp*Ap];
rankO = rank(KalO');
% Controllable and Observable, can place poles where ever we want

% Solving for feedback gains
fp1 = -10;
fp2 = -11;
Kx = place(Ap, Bp, [fp1 fp2]);
nAp = Ap-Bp*Kx;
eig(nAp)

% Solving for observer gains
op1 = -40;
op2 = -44;
L = place(Ap', Cp', [op1 op2]);
L = L';
nnAp = Ap - L*Cp;
eig(nnAp)
% Controller State Space
Ac = Ap - Bp*Kx - L*Cp;
Bc = L;
Cc = Kx; % feedback output u = -Kx
Dc = 0.*Cc*Bc;
sysC = ss(Ac, Bc, Cc, Dc);

% observer feedback
of = [Ap-Bp*Kx Bp*Kx; zeros(2) Ap-L*Cp];
eig(of);

% closed loop state space
sysCl = feedback(sysP, sysC);

% step response
time = 0:0.05:8;
u = ones(size(time));
[y, tout, x] = lsim(sysCl,u,time);
gain = 1/max(y);

figure(1);
hold on
plot(time,u)
plot(time,gain*y)
hold off

