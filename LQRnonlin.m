function dxdt = LQRnonlin(t, inputs)
% Nonlinear system of tailsitter drone
x1 = inputs(1);
x2 = inputs(2);
x3 = inputs(3);
x4 = inputs(4);
g = 9.81;
l = 1;
M = 1;
J = 1;
K = [0.707106781186547,1.098684113467810,-65.192024052026430,-10.009596597866787;
    0.707106781186547,1.098684113467810,65.192024052026430,10.009596597866787];
u = -K*[x1; x2; x3; x4]+[g/2; g/2];
F1 = u(1);
F2 = u(2);
eq1 = x2;
eq2 = ((F1+F2)/M)*cos(x3)-g;
eq3 = x4;
eq4 = ((F2-F1)/J)*l;
dxdt = [eq1; eq2; eq3; eq4];
end

