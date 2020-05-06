function K_list = lqr_finite(A,B,H,Q,R)
    K_list = cell(H,1);
    S = Q;
    for i = 1:H
        F = -(R+B'*S*B)\B'*S*A;
        S = Q + A'*S*A + A'*S*B*F;
        K_list{end-i+1} = F;
    end
end