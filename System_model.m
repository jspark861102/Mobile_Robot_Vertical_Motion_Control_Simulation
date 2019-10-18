function [An,Bn,Dn,Arn,Brn,Drn,M,C,K,F] = System_model;

%% parameters
global ms;
global Jy;
global Jw;
global a;
global b;
global h;
global ksf;
global ksr;
global bsf
global bsr;
global mw;
global kt;
global ktf;
global ktr;
global r;
global g;

%% M
M = zeros(3,3);

M(1,1) = ms + 4*mw;
M(2,2) = ms; 
M(3,1) = -4*mw*h; M(3,3) = Jy;

%% C
C = zeros(3,3);

C(2,2) = 2*(bsf + bsr); C(2,3) = -2*(a*bsf - b*bsr); 
C(3,2) = -2*(a*bsf - b*bsr); C(3,3) = 2*(a^2*bsf + b^2*bsr);

%% K
K = zeros(3,3);

K(2,2) = 2*(ksf + ksr); K(2,3) = -2*(a*ksf - b*ksr);
K(3,2) = -2*(a*ksf - b*ksr); K(3,3) = 2*(a^2*ksf + b^2*ksr); 

%% F
F = zeros(3,2);
F(1,1) = 2; F(1,2) = 2;
F(3,1) = -2*(h + r); F(3,2) = -2*(h + r);

%% D
DD = zeros(3,4);
DD(2,1) = 2*ksf; DD(2,2) = 2*ksr; DD(2,3) = 2*bsf; DD(2,4) = 2*bsr;
DD(3,1) = -2*a*ksf; DD(3,2) = 2*b*ksr; DD(3,3) = -2*a*bsf; DD(3,4) = 2*b*bsr;

%% M C K to A B matrix
A = [zeros(3,3) eye(3,3);-inv(M)*K -inv(M)*C];
B = [zeros(3,size(F,2));inv(M)*F];
D = [zeros(3,size(DD,2));inv(M)*DD];

%6DOF --> 5DOF
A = A(2:end,2:end);
B = B(2:end,:);
D = D(2:end,:);
An = A;Bn = B; Dn = D;

%% Rigid body model
Ar = [0]; 
Br = [2/(ms+4*mw)*ones(1,2)];
Dr = zeros(1,4);
Arn = Ar; Brn = Br; Drn = Dr;