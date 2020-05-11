function u_seq = robust_mpc(A,B,x0,H,ref,Q,R,d_min,d_max)
persistent options
if isempty(options)
    options = sdpsettings('robust.lplp','duality','verbose',0);
end
n = size(A,1);
m = size(B,2);
u_seq = zeros(m,H);
x = sdpvar(n,H);        % x1, x2, ..., x_H
u = sdpvar(m,H);       % u0, u1, ..., u_H-1
d = sdpvar(n,H);
objective = 0;

% uncertain d
W_D = [d_min<= d(:) <= d_max, uncertain(d)];

% dynamics contraints x^+ = Ax+Bu+Ed
W_dyn = [x(:,1) == A*x0 + B*u(:,1) + d(:,1)];
for i = 1:H-1
    W_dyn = [W_dyn, x(:,i+1) == A*x(:,i) + B*u(:,i+1) + d(:,i+1)];
    objective = objective + (x(:,i)-ref(:,i))'*Q*(x(:,i)-ref(:,i)) + u(:,i)'*R*u(:,i);
end
objective = objective + (x(:,H)-ref(:,H))'*Q*(x(:,H)-ref(:,H)) + u(:,H)'*R*u(:,H);

diagnostics = optimize(W_D+W_dyn,objective,options);
if diagnostics.problem == 0
    u_seq = value(u);
elseif diagnostics.problem == 1
    disp('Solver thinks it is infeasible')
else
    disp('Something else happened')
end

end