function [cost,x_traj,u_sig,r_traj]=simulator_run(tstep,Tspan,n,x0,A,B,H,Q,R,visual_flag,td, cont,  ref_sig, d_sig)
% The MPC simulator for integrator dynamics
% Author: Zexiang Liu
%% Parameters
N = Tspan/tstep+1;      % step number
if nargin == 11
    cont =  @(A,B,x,H,ref, Q,R,options) mpc_cont(A,B,x,H,ref,Q,R,options);
    ref_sig = @(t,n,H,tstep) ref_gen(t,n,H,tstep);
    d_sig =  @(t,n,tstep) d_gen(t,n,tstep);
end

%% Main Simulation Loop
x_traj = x0;
r_traj = zeros(n,1);
x = x0;
u_sig = [];
t = 0;
for i = 1:N-1
    t = t + tstep
    % MPC control
    ref = ref_sig(t,n,H,tstep);
    if size(x0,2)>1
        keyboard();
    end
    try
        [u_seq] = cont(A,B,x,H,ref,Q,R); 
    catch
        keyboard();
    end
    % state update
    try
        d = d_sig(t,n,tstep);% disturbance
    catch
        keyboard();
    end
    u = u_seq(:,1);
    x = A*x + B*u + d;
    
    % record
    x_traj = [x_traj,x];
    u_sig = [u_sig, u];
    r_traj = [r_traj,ref(:,1)];
    
    if visual_flag == 1
        if mod(i,td) == 0
            for k=1:n
                figure(k); hold on;
                plot([i-td:i]*tstep,x_traj(k,end-td:end),'b'); 
                plot([i-td:i]*tstep,r_traj(k,end-td:end),'r');
            end
            drawnow;
        end
    end
end

%% Evaluation
if visual_flag == 2
    for k = 1:n
        figure(k); hold off;
        plot([0:i]*tstep,x_traj(k,:),'b'); hold on;
        plot([0:i]*tstep,r_traj(k,:),'r');
    end
end
% compute total cost
cost = 0;
for i = 1:N-1
    cost = cost + x_traj(:,i)'*Q*x_traj(:,i) + u_sig(:,i)'*R*u_sig(:,i);
end
disp("The final cost is:"+num2str(cost));