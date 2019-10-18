function  [rho_from_input,rho_rigid_from_input,u,ur,Y,Yr] = InputandResponse(iWc,iWcr,A,B,D,Ar,Br,Dr,t,tr,Ts,xi,xf,xir,xfr,type,w,finite_time_of_DOC);

t_final = finite_time_of_DOC;

%u
u = zeros(type/2,length(t));
sys_dsum = ss(A,D,eye(length(A)),0);
[Y_dsum] = lsim(sys_dsum,w,t,zeros(length(A),1));
dsum = Y_dsum(end,:)';

for i = 1 : length(t)    
    u(:,i) = B'*expm(A'*(t_final-t(i)))*iWc*(xf-dsum);
end
if type/2 == 2
    rho_from_input = (sum(u(1,:).^2)+sum(u(2,:).^2))*Ts
else
    rho_from_input = sum(u(1,:).^2)*Ts;
end

%response
sys = ss(A,[B D],eye(length(A)),0);
[Y] = lsim(sys,[u;w],t,xi);

%ur
ur = zeros(type/2,length(t));
for i = 1 : length(t)
    ur(:,i) = Br'*expm(Ar'*(t_final-tr(i)))*iWcr*xfr;
end
if type/2 == 2
    rho_rigid_from_input = (sum(ur(1,:).^2)+sum(ur(2,:).^2))*Ts;
else
    rho_rigid_from_input = sum(ur(1,:).^2)*Ts;
end

%response
[Yr] = lsim(sys,[ur;w],tr,xi);