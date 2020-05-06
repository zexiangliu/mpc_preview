function tests = test_dyn
tests = functiontests(localfunctions);
end

function test_integrator(testCase)
% Test specific code
[A,B] = dyn(2);
A_t = [0 1;0 0];
B_t = [0;1];
verifyEqual(testCase,[A,B],[A_t,B_t]);
end
