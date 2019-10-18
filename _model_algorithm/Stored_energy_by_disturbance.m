function [Stored_potential_energy,Stored_kinetic_energy] = Stored_energy_by_disturbance(xi,Y,M,K,Ts,v_initial)

%v_initial »©±â
xi(3) = xi(3) - v_initial;
xff = Y(end,1:5)';
xff(3) = xff(3) - v_initial;

%
initial_state = [0;xi];
Positioin_initial_state = initial_state(1:3);
Velocity_initial_state = initial_state(4:6);

position = cumsum(Y(:,3))*Ts;
position =[0 ; position(1:end-1)];

Final_state = [position(end);xff];
Positioin_final_state = Final_state(1:3);
Velocity_final_state = Final_state(4:6);

Potential_energy_at_initialstate = 0.5*Positioin_initial_state'*K*Positioin_initial_state;
Kinetic_energy_at_initialstate = 0.5*Velocity_initial_state'*M*Velocity_initial_state;

Potential_energy_at_finalstate = 0.5*Positioin_final_state'*K*Positioin_final_state;
Kinetic_energy_at_finalstate = 0.5*Velocity_final_state'*M*Velocity_final_state;

% Stored_potential_energy = Potential_energy_at_finalstate - Potential_energy_at_initialstate
% Stored_kinetic_energy = Kinetic_energy_at_finalstate - Kinetic_energy_at_initialstate

Stored_potential_energy = Potential_energy_at_finalstate - Potential_energy_at_initialstate;
Stored_kinetic_energy = Kinetic_energy_at_finalstate - Kinetic_energy_at_initialstate;


