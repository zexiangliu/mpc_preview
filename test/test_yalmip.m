function tests = test_yalmip
tests = functiontests(localfunctions);
end

function test_integrator(testCase)

sdpvar x w
F = [x+w <= 1];
W = [-0.5 <= w <= 0.5, uncertain(w)];
objective = (x+1)^2;

sol1 = optimize(F + W,objective,sdpsettings('robust.lplp','duality'));
x1 = value(x);
sol2 = optimize(F + W,objective,sdpsettings('robust.lplp','duality'));
x2 = value(x);

verifyEqual(testCase,x1,x2)
end