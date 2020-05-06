function ref = ref_gen(t,n,H,dt)
% generate reference signal.
% INPUTS:  t --- current simulation
%          n --- system dimension
%          H --- preview horizon
ref = zeros(n,H);

for i=1:H
    ref(:,i) = 0*ones(n,1);%[cos((t+i*dt)/5);sin((t+(i+1)*dt)/5)]; %[1;1];
end

end