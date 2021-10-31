function gc_new = visualize_solution(gc, params, tau, contact, interaction_force)

t_ = 0:params.simulation_dt:params.control_dt;
N = length(t_);
force = zeros(3,1);
%% Simulator Loop

tau_net = tau;
for j = 1:N
    if contact == 1
        [tau_reaction, force, ~] = simulate_reaction_force(gc.q,gc.dq, params, interaction_force);
        tau_net = tau + tau_reaction;
    end
    [gc, ~] = Q2_forward_simulator_solution(gc, tau_net, params.simulation_dt); 
end

draw_plane(params);
draw_force(force, gc.q, params);
draw_leg(gc, params);

gc_new = gc;
end