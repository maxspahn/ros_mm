function J = costFunctionSimple(z, p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
q_dot = z(13:19);
x_des = p(4:6);
q_des = p(7:13);
u = z(11:12);

Wq = diag(ones(size(q, 1), 1) * 100);
Wx = diag(ones(size(x_des, 1), 1) * 100);
% Turning of rotation of base
Wx(3, 3) = 1;
Pu = diag(ones(size(u, 1), 1) * 0);
Pqdot = diag(ones(size(q_dot, 1), 1) * 1);

J = (x - x_des)' * Wx * (x - x_des) + ...
    (q - q_des)' * Wq * (q - q_des) + ...
    u' * Pu * u + ...
    q_dot' * Pqdot * q_dot;
    

end

