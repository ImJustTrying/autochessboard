close all;
clc;
figure("Position", [20 0 700 700]);
clf;

dtheta1 = pi;
dtheta2 = 2*pi;
l = 5*sqrt(2)/2;
l1 = l;
l2 = l;
t = 0;
dt = 0.01;
x0 = [0,0];
x1 = [l1,0];
x2 = [0,0];
theta1 = 0;
theta2 = pi;

for i = 1:200
  t = t + dt;
  theta1 = theta1 + dtheta1 * dt;
  theta2 = theta2 + dtheta2 * dt;
  x1 = x0 + [l1 * cos(theta1), l1 * sin(theta1)];
  x2 = x1 + [l2 * cos(theta2), l2 * sin(theta2)];
  drawLine2D(x0, x1);
  hold on;
  drawLine2D(x1, x2);
  axis([-5 5 -5 5]);
  grid on;
  xl = xlabel('regular text: $\theta$');
  set(xl, "Interpreter", "latex");
  set(xl, "fontsize", 15);
  ylabel('$y\  (m)$','interpreter','latex','fontsize',15);
  pause(0.001);
  hold off;
endfor
