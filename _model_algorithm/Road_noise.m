function [w,Sw] = Road_noise(roadnoise,a,b,v_initial,Ts,finite_time_of_DOC);

%% road noise
if roadnoise == 1
%     load road_noise_rand_50.mat;
%     road_noise = road_noise_rand_50;
    load w_rand_3.mat;
    road_noise = w_rand_3;

%     road_noise = 0.005*rand(length([0:Ts:finite_time_of_DOC]),1)';    
%     road_noise = road_noise - mean(road_noise);     
  
    zuf = road_noise;  
    if v_initial == 0
        zur = 0.005*rand(length([0:Ts:finite_time_of_DOC]),1)';        
    else        
        noise_lag = round(((a+b)/v_initial)/Ts);
        zur = [zeros(1,noise_lag) zuf(1:end-noise_lag)];        
    end
    dzuf = [0 diff(zuf)]/Ts;
    dzur = [0 diff(zur)]/Ts; 
       
    w = [zuf;zur;dzuf;dzur];
    
else
   w = zeros(4,length([0 :Ts: finite_time_of_DOC]));
end

%% calculate Sw of roadnoise
% if roadnoise == 1
%     Fs=100;
%     NFFT=64;%256;
%     NOVERLAP=0.5*NFFT;
%     WINDOW=hanning(NFFT);
%     dt=1/Fs;
%     dF=Fs/NFFT;
% 
%     zuf_f = zeros(NFFT,1); zur_f = zeros(NFFT,1); dzuf_f = zeros(NFFT,1); dzur_f = zeros(NFFT,1);
%     Szuf_zuf = zeros(NFFT,1); Szur_zur = zeros(NFFT,1); Sdzuf_dzuf = zeros(NFFT,1); Sdzur_dzur = zeros(NFFT,1);
%     Szuf_zur = zeros(NFFT,1); Szuf_dzuf = zeros(NFFT,1); Szuf_dzur = zeros(NFFT,1); 
%     Szur_dzuf = zeros(NFFT,1); Szur_dzur = zeros(NFFT,1); 
%     Sdzuf_dzur = zeros(NFFT,1);
% 
%     % The number of average
%     N=floor((length(zuf)-NFFT)/(NFFT-NOVERLAP))+1;
% 
%     % 2-sided spectral density functions
%     for n=1:N
%         zuf_f=fft(zuf(1+(n-1)*(NFFT-NOVERLAP):NFFT+(n-1)*(NFFT-NOVERLAP)))'/NFFT;
%         zur_f=fft(zur(1+(n-1)*(NFFT-NOVERLAP):NFFT+(n-1)*(NFFT-NOVERLAP)))'/NFFT;
%         dzuf_f=fft(dzuf(1+(n-1)*(NFFT-NOVERLAP):NFFT+(n-1)*(NFFT-NOVERLAP)))'/NFFT;
%         dzur_f=fft(dzur(1+(n-1)*(NFFT-NOVERLAP):NFFT+(n-1)*(NFFT-NOVERLAP)))'/NFFT;
% 
%         Szuf_zuf = Szuf_zuf + conj(zuf_f).*zuf_f/N/dF;
%         Szur_zur = Szur_zur + conj(zur_f).*zur_f/N/dF;
%         Sdzuf_dzuf = Sdzuf_dzuf + conj(dzuf_f).*dzuf_f/N/dF;
%         Sdzur_dzur = Sdzur_dzur + conj(dzur_f).*dzur_f/N/dF;
% 
%         Szuf_zur = Szuf_zur + conj(zuf_f).*zur_f/N/dF;
%         Szuf_dzuf = Szuf_dzuf + conj(zuf_f).*dzuf_f/N/dF;
%         Szuf_dzur = Szuf_dzur + conj(zuf_f).*dzur_f/N/dF;
%         Szur_dzuf = Szur_dzuf + conj(zur_f).*dzuf_f/N/dF;
%         Szur_dzur = Szur_dzur + conj(zur_f).*dzur_f/N/dF;
%         Sdzuf_dzur = Sdzuf_dzur + conj(dzuf_f).*dzur_f/N/dF;    
%     end
% 
%     % Correlation function
%     tau_corr = -(NFFT/Fs/2):dt:(NFFT/Fs/2)-dt;
% 
%     Rzuf_zuf = fftshift(ifft(Szuf_zuf,NFFT));
%     Rzur_zur = fftshift(ifft(Szur_zur,NFFT));
%     Rdzuf_dzuf = fftshift(ifft(Sdzuf_dzuf,NFFT));
%     Rdzur_dzur = fftshift(ifft(Sdzur_dzur,NFFT));
% 
%     Rzuf_zur = fftshift(ifft(Szuf_zur,NFFT));
%     Rzuf_dzuf = fftshift(ifft(Szuf_dzuf,NFFT));
%     Rzuf_dzur = fftshift(ifft(Szuf_dzur,NFFT));
%     Rzur_dzuf = fftshift(ifft(Szur_dzuf,NFFT));
%     Rzur_dzur = fftshift(ifft(Szur_dzur,NFFT));
%     Rdzuf_dzur = fftshift(ifft(Sdzuf_dzur,NFFT));
% 
% %     figure;plot(tau_corr,Rzuf_zuf)
% %     figure;plot(tau_corr,Rzur_zur)
% %     figure;plot(tau_corr,Rdzuf_dzuf)
% %     figure;plot(tau_corr,Rdzur_dzur)
% %     
% %     figure;plot(tau_corr,Rzuf_zur)
% %     figure;plot(tau_corr,Rzuf_dzuf)
% %     figure;plot(tau_corr,Rzuf_dzur)
% %     figure;plot(tau_corr,Rzur_dzuf)
% %     figure;plot(tau_corr,Rzur_dzur)
% %     figure;plot(tau_corr,Rdzuf_dzur) 
% 
% %     Sw = [max(abs(Rzuf_zuf)) max(abs(Rzuf_zur)) max(abs(Rzuf_dzuf)) max(abs(Rzuf_dzur));
% %         max(abs(Rzuf_zur)) max(abs(Rzur_zur)) max(abs(Rzur_dzuf)) max(abs(Rzur_dzur));
% %         max(abs(Rzuf_dzuf)) max(abs(Rzur_dzuf)) max(abs(Rdzuf_dzuf)) max(abs(Rdzuf_dzur));
% %         max(abs(Rzuf_dzur)) max(abs(Rzur_dzur)) max(abs(Rdzuf_dzur)) max(abs(Rdzur_dzur))];%/7.2;
%     Sw = diag([max(abs(Rzuf_zuf)) max(abs(Rzur_zur)) max(abs(Rdzuf_dzuf)) max(abs(Rdzur_dzur))]);%/7.2;
% %     Sw = diag([mean(Szuf_zuf) mean(Szur_zur) mean(Sdzuf_dzuf) mean(Sdzur_dzur)]);
% else
%     Sw = zeros(4,4);
% end

% Sw = cov(w');

if roadnoise == 1
     Sw = diag([cov(zuf);cov(zur);cov(dzuf);cov(dzur)]);
else
     Sw = zeros(4,4);
end
    