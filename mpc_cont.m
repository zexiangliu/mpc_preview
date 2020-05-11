function u_seq = mpc_cont(A,B,x0,H,ref,Q,R,options)
% Model predictive controller, return a sequence of control inputs.
% Collocation is used to construct the mpc.
% INPUTS: A,B --- system dynamics x+ = Ax + Bu
%         x   --- current state
%         H   --- preview horizon
%         ref --- reference trajectory, size=[n,H]
%         Q,R --- running cost x'Qx + u'Ru
if nargin == 7
    options = mpcqpsolverOptions;
end

n = size(A,1);          % state dim
m = size(B,2);          % input dim
N = H*(n+m);            % # decision variables
%helper functions
ind_x = @(k) n*(k-1)+(1:n); % index of x, k=1,...,H
ind_u = @(k) H*n + m*k + (1:m); % index of u, k=0,...,H-1
% Equality contraints: Aeq*z = beq
Aeq = [];
beq = [];
% x+ - Ax - Bu = 0
for k=1:H
   tmpA = zeros(n,N);
   tmpb = zeros(n,1);
   tmpA(:,ind_x(k)) = eye(n);
   tmpA(:,ind_u(k-1)) = -B;
   if k > 1
       tmpA(:,ind_x(k-1))= -A;
   else
       tmpb = A*x0;
   end
   Aeq = [Aeq;tmpA];
   beq = [beq;tmpb];

end
% Inequality constraints (placeholder)

% cost function: 
Q_aug = zeros(N,N);
f_aug = zeros(N,1);
for k=1:H
    Q_aug(ind_x(k),ind_x(k)) = Q;
    Q_aug(ind_u(k-1),ind_u(k-1)) = R;
    f_aug(ind_x(k)) = -Q*ref(:,k);
end
% call quadprog solver
% z = quadprog(Q_aug,f_aug,[],[],Aeq,beq,[],[],[],options);
L = chol(Q_aug,'lower');
Linv = inv(L);
[z,status] = mpcqpsolver(Linv,f_aug,[],zeros(0,1),Aeq,beq,false(0,1),options);
if status < 0
    error('infeasible problem or numerical error!');
end
% reshape output
if m == 1
    u_seq = z(H*n+1:end)';
else
    u_seq = reshape(z(H*n+1:end),[m,H]);
end
end