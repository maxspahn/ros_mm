function x_next = transitionFunctionAcc(z, p)

x_base = z(1:3);
u = z(4:5);
q = z(6:12);
slack = z(13);
q_dot = z(16:22);
u_dot = z(14:15);
dt = p(1);
r = p(2);
L = p(3);
x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
    x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
    x_base(3) + r/L * (u(1) - u(2)) * dt];
u_next = u + dt * u_dot;
q_next = q + dt * q_dot;

x_next = [x_base_next; u_next; q_next];
%x_next = RK4(z, p, @continuousDynamics, dt);
end

function x_dot = continuousDynamics(z, p)
    x_base = z(1:3);
    q_dot = z(13:19);
    u = z(11:12);
    r = p(2);
    L = p(3);
    x_base_dot = [r/2 * (u(1) + u(2)) * cos(x_base(3));...
        r/2 * (u(1) + u(2)) * sin(x_base(3));...
        r/L * (u(1) - u(2))];
    x_dot = [x_base_dot; q_dot];
end

