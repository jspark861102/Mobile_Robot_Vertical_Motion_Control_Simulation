function [rho_frequency] = DODR_frequency(A,B,D,iWc,Sw,xf,roadnoise);


%% DOC at the frequency domain

iWc_w = iWc/2/pi;

d_omega = 0.1;
omega = [0.1 : d_omega : 100];

sum_Hc = zeros(size(A));
sum_Hd = zeros(size(A));

for k = 1 : length(omega)
    Hc(:,:,k) = inv(j*omega(k)*eye(size(A))-A)*B*B'*inv(-j*omega(k)*eye(size(A))-A');
    Hd(:,:,k) = inv(j*omega(k)*eye(size(A))-A)*D*Sw*D'*inv(-j*omega(k)*eye(size(A))-A');
    
    sum_Hc = sum_Hc + Hc(:,:,k)*d_omega;
    sum_Hd = sum_Hd + Hd(:,:,k)*d_omega;
    
    rho_frequency(k) = trace(2*iWc_w*Hd(:,:,k));
end

if roadnoise == 1
    mag_rho_frequency = abs(rho_frequency);
    figure;
    semilogx(omega/2/pi,mag_rho_frequency,'--k','LineWidth',3)
    grid on
    set(gca,'Fontsize',16,'FontWeight','demi')
    xlabel('F (Hz)')
    ylabel('Input energy')
    grid on
    % hold on

    sum_rho_frequency = abs(trace(2*iWc_w*sum_Hd))
end

