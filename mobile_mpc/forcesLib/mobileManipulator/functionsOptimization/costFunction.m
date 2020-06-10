function J = costFunction(z, p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
g = p(4:13);
u = z(11:12);
qdot = z(13:19);

W = diag(ones(10, 1) * 100);
W(3, 3) = 0;
P = diag(ones(9, 1));

J = ([x; q] - g)' * W * ([x; q] - g) + [u; qdot]' * P * [u; qdot];

end

