function [Wd,Wc,Wcr,iWc,iWcr,rho,rho_rigid,t,tr] = DOCandDODR(A,B,D,Ar,Br,Dr,finite_time_of_DOC,Ts,Sw,xi,xf,xir,xfr);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Degree Of Controllability %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DOC for finite time
t_final = finite_time_of_DOC;

%by ode
%Wc
ini = zeros(1,length(A)^2);
[t,y] = ode45(@gra_AB,[0 :Ts: t_final],ini);
for i = 1 : length(A)
    for j = 1 : length(A)
        Wc(i,j) = y(end,j+(i-1)*length(A));
    end
end
rank_Wc = rank(Wc);
iWc = inv(Wc);

%Wd
ini = zeros(1,length(A)^2);
[t,yd] = ode45(@gra_AD,[0 :Ts: t_final],ini);
for i = 1 : length(A)
    for j = 1 : length(A)
        Wd(i,j) = yd(end,j+(i-1)*length(A));
    end
end
rank_Wd = rank(Wd);

% rho = (expm(A*t_final)*xi - xf)'*iWc*(expm(A*t_final)*xi - xf)
% rho_wo_d = (-xf)'*iWc*(-xf)
rho = trace(iWc*(xf*xf' + Wd))


% [T X] = ode45(@gra_AB_modified, [0 :Ts: t_final], ini, [], A, B);
% 
% [m n] = size(X);
% XX = mat2cell(X, ones(m,1), n);
% fh_reshape = @(x)reshape(x,size(A));
% XX = cellfun(fh_reshape,XX,'UniformOutput',false);
% global XX;

%% DOC with rigid body model
%by ode
inir = zeros(1,length(Ar)^2);
[tr,yr] = ode45(@gra_ArBr,[0 :Ts: t_final],inir);
for i = 1 : length(Ar)
    for j = 1 : length(Ar)
        Wcr(i,j) = yr(end,j+(i-1)*length(Ar));
    end
end
rank_Wcr = rank(Wcr);
iWcr = inv(Wcr);

%Wdr
ini = zeros(1,length(Ar)^2);
[t,ydr] = ode45(@gra_ArDr,[0 :Ts: t_final],ini);
for i = 1 : length(Ar)
    for j = 1 : length(Ar)
        Wdr(i,j) = ydr(end,j+(i-1)*length(Ar));
    end
end
rank_Wdr = rank(Wdr);

rho_rigid = trace(iWcr*(xfr*xfr' + Wdr))

% rho_rigid = (expm(Ar*t_final)*xir - xfr)'*iWcr*(expm(Ar*t_final)*xir - xfr)
% rho_rigid = (- xfr)'*iWcr*(-xfr);