% NOW-HOW
% Symbolic Math Toolbox
% If you want to do Mathematical Derivation 
% using MATLAB

% Defining the Symbolic Variables and General Coordinates
% x(t) - cart position
% th(t) - pendulum arm angle
% m - Pendulum Ball mass
% B_m - Pendulum Ball friction coefficient
% M - cart Mass
% B_M - Cart friction coefficient
% g - gravitational acceleration
% Force - force applied
clc
clear
close all

syms x(t) th(t) m M L B_M B_m g Force

%%
% Defining the ball X,Y coordinates
x_m = x+L*sin(th);
y_m = L*(1+cos(th));

% Printing x_m and y_m
fprintf('%s\n', repmat('-', 1, 100));

fprintf('<strong>%s</strong>','Ball X Coordinates (x_m): ')
fprintf('\n')
pretty(x_m)
fprintf('\n')

fprintf('<strong>%s</strong>','Ball Y Coordinates (y_m): ')
fprintf('\n')
pretty(y_m)
fprintf('\n')



%%
% Getting derivative of the generalized coordinates
% xdot is velocity
% thdot is angular velocity

% Does not matter how we write because x is dependent only on t 
% if x was dependent on t and z ( syms x(t,z) ) we would have to specify which variable to take
% derivative about
% diff(x) = diff(x,t)
xdot = diff(x);
thdot = diff(th,t);

% Taking derivative of the ball position
xdot_m = diff(x_m);
ydot_m = diff(y_m);

fprintf('<strong>%s</strong>','Ball Velocity X-component: ')
fprintf('\n')
pretty(xdot_m)
fprintf('\n')

fprintf('<strong>%s</strong>','Ball Velocity Y-component: ')
fprintf('\n')
pretty(ydot_m)
fprintf('%s', repmat('-', 1, 100));
fprintf('\n')


%%
% Energy States
Kinetic_Energy = 1/2*M*xdot^2 + 1/2*m*(xdot_m^2+ydot_m^2);
% Displaying the Energy States
fprintf('<strong>%s</strong>','Kinetic Energy: ')
fprintf('\n')
fprintf('\n')
pretty(simplify(Kinetic_Energy))
fprintf('\n')


Potential_Energy = m*g*y_m;
fprintf('<strong>%s</strong>','Potential Energy: ')
fprintf('\n')
fprintf('\n')
pretty(Potential_Energy)
fprintf('\n')



Dissipated_Energy = 1/2*B_M*xdot^2 + 1/2*B_m*thdot^2;
fprintf('<strong>%s</strong>','Dissipated Energy: ')
fprintf('\n')
fprintf('\n')
pretty(Dissipated_Energy)
fprintf('\n')
fprintf('%s\n', repmat('-', 1, 100));
fprintf('\n')

% Lagrange Equations
% Lagrange for Generalized Coordinate q1 = X
% Finding Lagrange Relative x generalized coordinate
dK_dxdot = diff(Kinetic_Energy,xdot);
dt_dK_dxdot = diff(dK_dxdot,t);

fprintf('<strong>%s</strong>','First Term of Lagrange (dt_dK_dxdot): ')
fprintf('\n')
fprintf('\n')
pretty(dt_dK_dxdot)
fprintf('\n')

dK_dx = diff(Kinetic_Energy,x);

fprintf('<strong>%s</strong>','Second Term of Lagrange (dK_dx): ')
fprintf('\n')
fprintf('\n')
pretty(dK_dx)
fprintf('\n')

dD_dxdot = diff(Dissipated_Energy,xdot);

fprintf('<strong>%s</strong>','Third Term of Lagrange (dD_dxdot): ')
fprintf('\n')
fprintf('\n')
pretty(dD_dxdot)
fprintf('\n')

dP_dx = diff(Potential_Energy, x);

fprintf('<strong>%s</strong>','Fourth Term of Lagrange (dP_dx): ')
fprintf('\n')
fprintf('\n')
pretty(dP_dx)
fprintf('\n')
fprintf('%s\n', repmat('-', 1, 100));
fprintf('\n')

% Lagrange Equations
Lagrange_x = dt_dK_dxdot - dK_dx + dD_dxdot + dP_dx == Force;

fprintf('<strong>%s</strong>','Lagrange X: ')
fprintf('\n')
fprintf('\n')
pretty(isolate(simplify(Lagrange_x),Force))
fprintf('\n')
fprintf('<strong>%s</strong>','Without Isolate and Simplify: ')
fprintf('\n')
pretty(Lagrange_x)
fprintf('\n')


fprintf('<strong>%s</strong>','Isolating X'''': ')
fprintf('\n')
pretty(isolate(simplify(Lagrange_x),diff(x,t,t)))
fprintf('\n')
fprintf('%s\n', repmat('-', 1, 100));
fprintf('\n')

% Isolating the x'' and getting only the right hand side
xdotdot = rhs(isolate(Lagrange_x,diff(x(t),t,t)));

% Lagrange for Generalized Coordinate q2 = θ
% Finding Lagrange Relative theta generalized coordinate
dK_dthdot = diff(Kinetic_Energy,thdot);
dt_dK_dthdot = diff(dK_dthdot,t);

fprintf('<strong>%s</strong>','First Term of Lagrange (dt_dK_dthdot): ')
fprintf('\n')
fprintf('\n')
pretty(dt_dK_dthdot)
fprintf('\n')

dK_dth = diff(Kinetic_Energy,th);

fprintf('<strong>%s</strong>','Second Term of Lagrange (dK_dth): ')
fprintf('\n')
fprintf('\n')
pretty(dK_dth)
fprintf('\n')

dD_dthdot = diff(Dissipated_Energy,thdot);

fprintf('<strong>%s</strong>','Third Term of Lagrange (dD_dthdot): ')
fprintf('\n')
fprintf('\n')
pretty(dD_dthdot)
fprintf('\n')

dP_dth = diff(Potential_Energy, th);

fprintf('<strong>%s</strong>','Fourth Term of Lagrange (dP_dth): ')
fprintf('\n')
fprintf('\n')
pretty(dP_dth)
fprintf('\n')
fprintf('%s\n', repmat('-', 1, 100));
fprintf('\n')

Lagrange_th = dt_dK_dthdot - dK_dth + dD_dthdot + dP_dth == 0;

fprintf('\n')
fprintf('<strong>%s</strong>','Lagrange theta: ')
fprintf('\n')
pretty(0 == lhs(simplify(Lagrange_th))-rhs(simplify(Lagrange_th)))
fprintf('\n')
fprintf('<strong>%s</strong>','Isolating theta'''': ')
fprintf('\n')
pretty(isolate(simplify(Lagrange_th),diff(th,t,t)))
fprintf('%s\n', repmat('-', 1, 100));
fprintf('\n')

% Isolating the θ'' and getting only the right hand side
thdotdot = rhs(isolate(Lagrange_th,diff(th(t),t,t)));

% Substituting θ'' in Lagrange_x and X'' in Lagrange_th
Lagrange_X_final = subs(Lagrange_x,diff(th(t),t,t),thdotdot);
pretty(simplify(Lagrange_X_final))


assume(L~=0)
sympref('AbbreviateOutput', false);  % Try to avoid abbreviating outputs
Lagrange_X_final = isolate(simplify(Lagrange_X_final),diff(x,t,t));

Lagrange_th_final = subs(Lagrange_th,diff(x(t),t,t),xdotdot);
assume(M+m~=0)
Lagrange_th_final = isolate(simplify(Lagrange_th_final),diff(th,t,t));

% isolate(simplify(Lagrange_X_final),diff(x,t,t))
fprintf('<strong>%s</strong>','Final X'''': ')
fprintf('\n')
pretty(Lagrange_X_final)


fprintf('\n')
fprintf('<strong>%s</strong>','Final Theta'''': ')
fprintf('\n')
pretty(Lagrange_th_final)
fprintf('%s', repmat('-', 1, 100));
fprintf('\n')

% Linearizing
%%%%%%%%%%%%
cauchy_eqn = [diff(x(t),t);
              rhs(Lagrange_X_final);
              diff(th(t),t);
              rhs(Lagrange_th_final)];

% Jacobian matrices
x1 = x;
x2 = diff(x(t),t);
x3 = th;
x4 = diff(th(t),t);

J_A = [diff(cauchy_eqn(1),x1), diff(cauchy_eqn(1),x2), diff(cauchy_eqn(1),x3), diff(cauchy_eqn(1),x4);
       diff(cauchy_eqn(2),x1), diff(cauchy_eqn(2),x2), diff(cauchy_eqn(2),x3), diff(cauchy_eqn(2),x4);
       diff(cauchy_eqn(3),x1), diff(cauchy_eqn(3),x2), diff(cauchy_eqn(3),x3), diff(cauchy_eqn(3),x4);
       diff(cauchy_eqn(4),x1), diff(cauchy_eqn(4),x2), diff(cauchy_eqn(4),x3), diff(cauchy_eqn(4),x4)];

% X is a free variable so our solution is independent
% Substitute 
J_down = subs(J_A,[x1 x2 x3 x4],[x1 0 pi 0]);
J_up = subs(J_A,[x1 x2 x3 x4],[x1 0 0 0]);
% pretty(simplify(J))

fprintf('<strong>%s</strong>','Jacobian with theta = 0 (UP)')
fprintf('\n')
disp(simplify(J_up))
fprintf('\n')
fprintf('<strong>%s</strong>','Jacobian with theta = PI (DOWN)')
fprintf('\n')
disp(simplify(J_down))

J_b = [diff(cauchy_eqn(1),Force);
       diff(cauchy_eqn(2),Force);
       diff(cauchy_eqn(3),Force);
       diff(cauchy_eqn(4),Force)];
J_b = subs(J_b,th(t),0);

fprintf('<strong>%s</strong>','Jacobian for Input Matrix')
fprintf('\n')
disp(J_b)
fprintf('%s\n', repmat('-', 1, 100));