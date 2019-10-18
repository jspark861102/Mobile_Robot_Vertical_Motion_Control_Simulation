clc
clear all
close all

A = -1;
B = 1;
R = 100;
Q = 10;
Qf=0;






%% solution of riccati equation

t_inverse = [4 : -0.001 : 0];

[T X] = ode45(@mRiccati_sensitivity, t_inverse, Qf, [], A, B, Q, R);
X(end)

%%
H = [A,-B*inv(R)*B';inv(Q) -A']
eig(H)