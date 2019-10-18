close all
clear all
clc

%% parameters
T = 0.001;                    % sec
v_in = 0.5;                     % m/s  
v_fn = 0;                     % m/s

type = 4;
actuator_dynamics_on = 1;           %apply actuaotr dynamics=1, otherwise 0
cutoff_frequency_of_actuator = 5;   % Hz

%% parameters
vehicle_parameters_AclassHatchback;
vehicle_parameters_RobotisMobileManipulator;
global ms; global Jy; global Jw; global a; global b; global h;
global ksf; global ksr; global bsf; global bsr; global mw; global kt;
global ktf; global ktr; global r; global g;

%% Make system model
[An,Bn,Dn,Arn,Brn,Drn,M,C,K,F] = System_model;
%X = [z theta V zdot thetadot]
%U = [Ftf Ftr]

%% reference
%XM430-W210-T torque : 3Nm
%open loop brake torque : 2Nm * four actuators
Tb_op = -0.2*4;

%(vfn - vin) = Tb_op/(mr)*t_op
a_op = Tb_op / ((ms+4*mw) * r);
t_op = (v_fn - v_in) / a_op; 

t = [0 : T : t_op];
v_ref = v_in + a_op*t;
% figure;plot(t,v_ref)
xin = [0 0 v_in 0 0]';
xfn = zeros(length(t), length(An));
xfn(:,3) = v_ref;
xfnend = xfn(end,:)';

xirn = v_in;
xfrnend = v_ref(end);

%% Actuator - 1st order low pass filter
[A_actuator,B_actuator,D_actuator,xi_actuator,xf_actuator,Ar_actuator,Br_actuator,Dr_actuator,xir_actuator,xfr_actuator] = Include_actuator(actuator_dynamics_on,type,An,Bn,Dn,Arn,Brn,Drn,xin,xfnend,xirn,xfrnend,cutoff_frequency_of_actuator);
if actuator_dynamics_on == 1
    A = A_actuator; B = B_actuator; D = D_actuator; 
    Ar = Ar_actuator; Br = Br_actuator; Dr = Dr_actuator;
    xi = xi_actuator; xf = xf_actuator;
    xir = xir_actuator; xfr = xfr_actuator;
else
    A = An; B = Bn; D = Dn;
    Ar = Arn; Br = Brn; Dr = Drn;
    xi = xin; xf = xfnend;
    xir = xirn; xfr = xfrnend;
end

%% analysis of nominal system
% [eigenvector, eigenvalue, rank_3DOF_Model_5by5] = Analysis_nominal_system(An,Bn,Dn);
sys = ss(A,B,eye(length(A)),0);
eig_op = eig(A);

sys_r = ss(Ar, Br,eye(length(Ar)),0);
eig_r_op = eig(Ar);

%% LQR
%X =     [z theta V   zdot   thetadot]
if actuator_dynamics_on == 1
%     Q = diag([100000 100000 1000 10000  10000 0 0]); %couple term Ʃ�� �ϸ�?
    Q = diag([100000 100000 1000 10000  10000 0 0]); %couple term Ʃ�� �ϸ�?
else
    Q = diag([100000 100000 1000 10000  10000]); %couple term Ʃ�� �ϸ�?
end
R = eye(2);
[Kgain, S, eig_cl] = lqr(sys, Q, R, 0);
eign = [eig_op eig_cl]
wn_cl = (abs(eig_cl))/2/pi;
wd_cl = abs(imag(eig_cl))/2/pi;

if actuator_dynamics_on == 1
    Qr = diag([1000 0 0]);
else
    Qr = 1000;
end
Rr = eye(2);
[Kgain_r, Sr, eig_r_cl] = lqr(sys_r, Qr, Rr, 0);
eigr = [eig_r_op eig_r_cl]

wn_r_cl = (abs(eig_r_cl))/2/pi;
wd_r_cl = abs(imag(eig_r_cl))/2/pi;

%% run simulink
sim('LQR_simulink')

%% plot
%real front input
figure;
plot(tout,input_wo.signals.values(:,1),'b', 'LineWidth', 1.5)
hold on
plot(tout,input_w.signals.values(:,1),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('real front input')
xlabel('t(sec)'); ylabel('force(N)')

%dynamics filtered input
figure;
plot(states_wo.signals.values(:,6),'b', 'LineWidth', 1.5)
hold on
plot(states_w.signals.values(:,6),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('filtered input')
xlabel('t(sec)'); ylabel('force(N)')


%z
figure;
plot(tout,states_wo.signals.values(:,1),'b', 'LineWidth', 1.5)
hold on
plot(tout,states_w.signals.values(:,1),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('z')
xlabel('t(sec)'); ylabel('force(N)')

%theta
figure;
plot(tout,states_wo.signals.values(:,2),'b', 'LineWidth', 1.5)
hold on
plot(tout,states_w.signals.values(:,2),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('theta')
xlabel('t(sec)'); ylabel('force(N)')

%V
figure;
plot(tout,states_wo.signals.values(:,3),'b', 'LineWidth', 1.5)
hold on
plot(tout,states_w.signals.values(:,3),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('V')
xlabel('t(sec)'); ylabel('force(N)')

%dz
figure;
plot(tout,states_wo.signals.values(:,4),'b', 'LineWidth', 1.5)
hold on
plot(tout,states_w.signals.values(:,4),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('dz')
xlabel('t(sec)'); ylabel('force(N)')

%dtheta
figure;
plot(states_wo.signals.values(:,5),'b', 'LineWidth', 1.5)
hold on
plot(states_w.signals.values(:,5),'--r', 'LineWidth', 1.5)
legend('without suspension', 'with suspension')
grid on
title('dtheta')
xlabel('t(sec)'); ylabel('force(N)')

