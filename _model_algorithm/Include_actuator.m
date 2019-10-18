function [A_actuator,B_actuator,D_actuator,xi_actuator,xf_actuator,Ar_actuator,Br_actuator,Dr_actuator,xir_actuator,xfr_actuator] = Include_actuator(actuator_dynamics_on,type,An,Bn,Dn,Arn,Brn,Drn,xin,xfn,xirn,xfrn,cutoff_frequency_of_actuator);

%% Actuator - 1st order low pass filter
%cut-off frequency of ICEV < 5Hz
%cut-off frequency of EV > 100Hz

f = cutoff_frequency_of_actuator;
tau = 1/2/pi/f;
sys_actuator = tf([1],[tau 1]);

if actuator_dynamics_on == 1
    A_actuator = [An Bn;zeros(type/2,length(An)) eye(type/2,type/2)*-1/tau];
    B_actuator = [zeros(5,type/2);eye(type/2,type/2)*1/tau];
    D_actuator = [Dn;zeros(type/2,4)];   
    xi_actuator = [xin' zeros(1,type/2)]';
    xf_actuator = [xfn' zeros(1,type/2)]';
else
    A_actuator = 0;
    B_actuator = 0;
    D_actuator = 0;   
    xi_actuator = 0';
    xf_actuator = 0';
end

if actuator_dynamics_on == 1
    % actuator
    Ar_actuator = [Arn Brn;zeros(type/2,length(Arn)) eye(type/2,type/2)*-1/tau];
    Br_actuator = [zeros(1,type/2);eye(type/2,type/2)*1/tau];
    Dr_actuator = [Drn;zeros(type/2,4)];  
    xir_actuator = [xirn' zeros(1,type/2)]';
    xfr_actuator = [xfrn' zeros(1,type/2)]';
else
    Ar_actuator = 0;
    Br_actuator = 0;
    Dr_actuator = 0;  
    xir_actuator = 0;
    xfr_actuator = 0;
end