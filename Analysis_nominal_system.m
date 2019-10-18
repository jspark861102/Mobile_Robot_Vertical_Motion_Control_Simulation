function [eigenvector, eigenvalue, rank_3DOF_Model_5by5] = Analysis_nominal_system(An,Bn,Dn)

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

%% analysis of nominal system
%nominal system
sys_n = ss(An,[Bn Dn],eye(length(An)),0);

%natural frequency and mode shape
[eigenvector,eigenvalue] = eig(An);
natural_frequency = abs(eigenvalue)/2/pi;
damped_natural_frequency = abs(imag(eigenvalue))/2/pi;
mode_shape = [eigenvector(:,1)/eigenvector(1,1) eigenvector(:,2)/eigenvector(1,2) eigenvector(:,3)/eigenvector(1,3) eigenvector(:,4)/eigenvector(1,4) eigenvector(:,5)];

% controllability of nominal system
rank_3DOF_Model_5by5 = rank(ctrb(An,Bn));

%bounce natural frquency - quarter car model
k_quartercar = ((2*ksf+2*ksr)*4*kt)/((2*ksf+2*ksr)+4*kt);
bounce_natural_frequency = sqrt(k_quartercar/ms)/2/pi;
zeta_quartercar = (2*bsf+2*bsr)/2/sqrt(ms*k_quartercar); %for good ride, zeta falls btw 0.2 and 0.4
bounce_damped_natural_frequency = bounce_natural_frequency * sqrt(1-zeta_quartercar^2);

% find predominant modes - if z/theta is positive, oscillation ceter will be ahead of the CG
alpha = 2*(ksf + ksr)/ms;
beta = 2*(a*ksf - b*ksr)/ms;
gamma = 2*(a^2*ksf + b^2*ksr)/Jy;
k = sqrt(Jy/ms);
omega1 = sqrt((alpha+gamma)/2 + sqrt(((alpha-gamma)^2)/4+(beta^2)/(k^2)));
omega2 = sqrt((alpha+gamma)/2 - sqrt(((alpha-gamma)^2)/4+(beta^2)/(k^2)));
z_over_theta11 = -beta/(alpha-(omega1)^2);
z_over_theta21 = -sqrt(Jy/ms)^2*(gamma-(omega1)^2)/beta;
z_over_theta12 = -beta/(alpha-(omega2)^2);
z_over_theta22 = -sqrt(Jy/ms)^2*(gamma-(omega2)^2)/beta;
if z_over_theta11 > 0 
    distance1 = a - (z_over_theta11+z_over_theta21)/2;
else
    distance1 = (z_over_theta11+z_over_theta21)/2 - b;
end
if z_over_theta12 > 0
    distance2 = a - (z_over_theta12+z_over_theta22)/2;
else
    distance2 = (z_over_theta12+z_over_theta22)/2 - b;
end
% if in = 1, then 
% if distance1 >0
%     pitch1 = 1
% else 
%     heave1 = 1
% end
% if distance2 >0
%     pitch2 = 1
% else 
%     heave2 = 1
% end
