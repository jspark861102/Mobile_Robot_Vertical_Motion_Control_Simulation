function  [rho_from_input,rhon_from_input,u,ur,Y,Yr] = InputandResponse1(iWc,iWcr,A,B,D,Ar,Br,Dr,t,tr,Ts,xi,xf,xir,xfr,w,finite_time_of_DOC);

t_final = finite_time_of_DOC;

%u
u = zeros(1,length(t));
% u = zeros(length(A),length(A),length(t));
sys_dsum = ss(A,D,eye(length(A)),0);
[Y_dsum] = lsim(sys_dsum,w,t,zeros(length(A),1));
dsum = Y_dsum(end,:)';

for i = 1 : length(t)    
    u(:,i) = B'*expm(A'*(t_final-t(i)))*iWc*(xf-dsum);
%     u = expm(A'*(t_final-t(i)));
end
rho_from_input = sum(u(1,:).^2)*Ts;

%response
sys = ss(A,[B D],eye(length(A)),0);
[Y] = lsim(sys,[u;w],t,xi);
% Y=0;

%ur
ur = zeros(1,length(t));
% ur = zeros(length(Ar),length(Ar),length(t));
sysn_dsum = ss(Ar,Dr,eye(length(Ar)),0);
[Yn_dsum] = lsim(sysn_dsum,w,tr,zeros(length(Ar),1));
dsumn = Yn_dsum(end,:)';
for i = 1 : length(t)
    ur(:,i) = Br'*expm(Ar'*(t_final-tr(i)))*iWcr*(xfr-dsumn);
%     ur = expm(Ar'*(t_final-tr(i)));
end
rhon_from_input = sum(ur(1,:).^2)*Ts;

%response
sysn = ss(Ar,[Br Dr],eye(length(Ar)),0);
[Yr] = lsim(sysn,[ur;w],tr,xir);
% Yr=0;