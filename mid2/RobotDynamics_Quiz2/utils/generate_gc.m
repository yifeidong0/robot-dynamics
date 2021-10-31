%% Generalized coordinates
function gc = generate_gc()

    % generate generalized coordinate symbolic variables
    syms q1 q2 q3 'real';
    syms dq1 dq2 dq3 'real';

    % Return struct with generalized coordinates and velocities
    gc.q = [q1 q2 q3]';
    gc.dq = [dq1 dq2 dq3]';

end