function z_dot = statespace(~, z)

global  Fy_total Fxd m Md Mc;

Iz = 13201;
z_dot = zeros(6,1);

z_dot(1) = z(3)*cos(z(5)) - z(4)*sin(z(5));
z_dot(2) = z(3)*sin(z(5)) + z(4)*cos(z(5));
z_dot(3) = Fxd/m+z(4)*z(6);
z_dot(4) = Fy_total/m-z(3)*z(6);
z_dot(5) = z(6);
z_dot(6) = (Md+Mc)/Iz;





