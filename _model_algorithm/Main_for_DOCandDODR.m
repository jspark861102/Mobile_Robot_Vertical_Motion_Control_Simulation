close all
clear all
clc

%% parameters
% code parameters
type = 4;                           % AWD:4, 2WD:2
roadnoise = 0;                      %apply roadnoise=1, otherwise 0
dataplot = 1;                       %plot data=1, otherwise 0
actuator_dynamics_on = 1;           %apply actuaotr dynamics=1, otherwise 0

% design parameters
cutoff_frequency_of_actuator = 1;   % Hz
finite_time_of_DOC = 3;             % sec  
Ts = 0.01;                          % sec
v_initial = 10;                     % m/s  
v_final = 20;                       % m/s

%% parameters
% vehicle_parameters_TaehyunShim;
% vehicle_parameters_EclassSedan;
% vehicle_parameters_EclassSUV;
vehicle_parameters_AclassHatchback;

global ms; global Jy; global Jw; global a; global b; global h;
global ksf; global ksr; global bsf; global bsr; global mw; global kt;
global ktf; global ktr; global r; global g;

%% Make system model
[An,Bn,Dn,Arn,Brn,Drn,M,C,K,F] = System_model(type);
xin = [0 0 v_initial 0 0]';
xfn = [0 0 v_final-v_initial 0 0]';
% xfn = [0.1 0 0 0 0]';
xirn = [v_initial]';
xfrn = [v_final-v_initial]';
% xfrn = [0]';

%% cost function
% cost function
Qn = zeros(5,5); Qr = zeros(3,3);  

Q = [0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0
    0 0 0 0 0 0 0];
% Q = 100000000000000000000*eye(7,7);Q(3,3) = 100;Q(6:7,6:7) = zeros(2,2);

Qfn = eye(5,5); Qfr = eye(3,3); 


% alpha = 100000000000000000000;
alpha = 10^18;
% alpha = 1;

Qf1 = [K(2:3,2:3) zeros(2,3);zeros(3,2) M];
Qf = zeros(7,7);
Qf(1:5,1:5) = alpha * Qf1;

%% analysis of nominal system
Analysis_nominal_system(An,Bn,Dn,v_initial,v_final,xin,xfn);

%% Make roadnoise
[w,Sw] = Road_noise(roadnoise,a,b,v_initial,Ts,finite_time_of_DOC);

%% Actuator - 1st order low pass filter
[A_actuator,B_actuator,D_actuator,xi_actuator,xf_actuator,Ar_actuator,Br_actuator,Dr_actuator,xir_actuator,xfr_actuator] = Include_actuator(actuator_dynamics_on,type,An,Bn,Dn,Arn,Brn,Drn,xin,xfn,xirn,xfrn,cutoff_frequency_of_actuator);

if actuator_dynamics_on == 1
    A = A_actuator; B = B_actuator; D = D_actuator; 
    Ar = Ar_actuator; Br = Br_actuator; Dr = Dr_actuator;
    xi = xi_actuator; xf = xf_actuator;
    xir = xir_actuator; xfr = xfr_actuator;
else
    A = An; B = Bn; D = Dn;
    Ar = Arn; Br = Brn; Dr = Drn;
    xi = xin; xf = xfn;
    xir = xirn; xfr = xfrn;
end
save A.mat A;save B.mat B; save D.mat D; save Sw.mat Sw;
save Ar.mat Ar;save Br.mat Br; save Dr.mat Dr;

%% DOC and DODR
[Wd,Wc,Wcr,iWc,iWcr,rho,rho_rigid,t,tr] = DOCandDODR(A,B,D,Ar,Br,Dr,finite_time_of_DOC,Ts,Sw,xi,xf,xir,xfr);
% [rho_frequency] = DODR_frequency(A,B,D,iWc,Sw,xf,roadnoise);
[rho_from_input,rho_rigid_from_input,u,ur,Y,Yr] = InputandResponse(iWc,iWcr,A,B,D,Ar,Br,Dr,t,tr,Ts,xi,xf,xir,xfr,type,w,finite_time_of_DOC);

%% Cehck Controllability
R_AB = rank(ctrb(A,B));
R_Wc = rank(Wc);

%% DODR modification
% [rho_new1,xf_remaining,rho_remaining,xf_openloop] = Cal_u_remaining(finite_time_of_DOC,Y,Yr,t,A,B,iWc,type,rho,rho_from_input,Ts,xf,v_initial);
% [Stored_potential_energy,Stored_kinetic_energy] = Stored_energy_by_disturbance(xi,Y,M,K,Ts,v_initial);

%% Stochastic Regulator
% [Y_SR,cost_SR,P0_SR,P_list_SR,sum_us,final_SR,us,YY] = StochasticRegulator(A,B,D,finite_time_of_DOC,Sw,xi,xf,Q,Qf,w,Y);

%% Terminal Control
% [Y_TC,cost_TC,P0_TC,S_list_TC] = TerminalControl(A,B,D,finite_time_of_DOC,Sw,xi,xf,Q,Qf,w,Y);
% [Y_LV,u_LV,cost_LV,sum_u_LV] = TerminalControl1(A,B,D,finite_time_of_DOC,Sw,xi,w,Y);

%% data plot
DATA_Plot(dataplot,actuator_dynamics_on,type,roadnoise,u,ur,Y,Yr,w,Ts,t,tr);
% DATASET_Plot;

%% prompt% clc
% 
% rho
% rho_from_input
% 
% cost_SR
% sum_u_SR = sum_us
% final_SR
% 
% cost_LV
% sum_u_LV



