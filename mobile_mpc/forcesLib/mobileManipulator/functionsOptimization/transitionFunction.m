function x_next = transitionFunction(z, p)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
u = z(11:12);
qdot = z(13:19);
dt = p(1);
r = p(2);
L = p(3);
x_next = [x(1) + r/2 * (u(1) + u(2)) * cos(x(3)) * dt;
    x(2) + r/2 * (u(1) + u(2)) * sin(x(3)) * dt;
    x(3) + r/L * (u(1) - u(2)) * dt;
    q(1) + dt * qdot(1);
    q(2) + dt * qdot(2); 
    q(3) + dt * qdot(3);
    q(4) + dt * qdot(4); 
    q(5) + dt * qdot(5); 
    q(6) + dt * qdot(6); 
    q(7) + dt * qdot(7)];
end

