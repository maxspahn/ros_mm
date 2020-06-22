function J = costFunctionAcc(z, p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
u = z(4:5);
q = z(6:12);
slack = z(13);
u_dot = z(14:15);
q_dot = z(16:22);
x_des = p(4:6);
q_des = p(7:13);

wq = p(14);
wx = p(15);
wo = p(16);
wslack = p(17);
wpu = p(18);
wpqdot = p(19);
wudot = p(20);

Wq = diag(ones(size(q, 1), 1) * wq);
Wx = diag(ones(size(x_des, 1), 1) * wx);
% Turning of rotation of base
Wx(3, 3) = wo;
Wslack = wslack;
Wu = diag(ones(size(u, 1), 1) * wpu);
Pqdot = diag(ones(size(q_dot, 1), 1) * wpqdot);
% avoid Turning cost
Pudot = diag(ones(size(u_dot, 1), 1) * wudot);
J = (x - x_des)' * Wx * (x - x_des) + ...
    (q - q_des)' * Wq * (q - q_des) + ...
    u' * Wu * u + ...
    u_dot' * Pudot * u_dot + ...
    q_dot' * Pqdot * q_dot + ...
    slack * Wslack * slack;
end

