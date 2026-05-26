% =========================================================
%  Configuración cinemática 
% =========================================================
clear; close all; clc;

%% --- Parámetros ---
syms a l d real

% Geometría del chasis
dx = [l, -l, -l,  l];   % offsets x de cada llanta
dy = [d,  d, -d, -d];   % offsets y de cada llanta

% Parámetros de cada llanta
phi   = [-pi/4,  pi/4, -pi/4,  pi/4];  % ángulo del rodillo
theta = [0, 0, 0, 0];                  % orientación de llanta en {B}
ai    = [a, a, a, a];                  % radio de cada llanta

%% --- Construcción de omega_i usando fn_wi ---
omega_i = sym(zeros(4,3));

for i = 1:size(phi, 2)
    omega_i(i,:) = fn_wi(ai(i), phi(i), theta(i), dx(i), dy(i));
end

disp('omega_i simbólica:')
disp(omega_i)

%% --- Pseudo-inversa ---
W = inv(omega_i.' * omega_i) * omega_i.';
W = simplify(W);

disp('W simbólica:')
disp(W)

%% --- Evaluación numérica ---
a_val = 5;  l_val = 20;  d_val = 10;

W  = double(subs(W,        [a, l, d], [a_val, l_val, d_val]));
dx = double(subs(dx,       [a, l, d], [a_val, l_val, d_val]));
dy = double(subs(dy,       [a, l, d], [a_val, l_val, d_val]));

disp('W numérica:'), disp(W)
disp('dx:'), disp(dx)
disp('dy:'), disp(dy)

%% --- Verificación ---
omega_num = double(subs(omega_i, [a, l, d], [a_val, l_val, d_val]));
disp('Verificación omega_i * W (debe ser I_3):')
disp(round(omega_num * W, 6))


% =========================================================
%  FUNCIÓN: fn_wi
%  Implementa la fórmula general de la llanta:
%
%  w_i = [1/a, (1/a)*tan(phi)] * R(theta_B) * J * [u;v;r]
%
%  Retorna la fila i de la matriz cinemática (1x3)
% =========================================================
function row = fn_wi(a_i, phi_i, theta_i, dx_i, dy_i)

    % Vector de rodillo (1x2)
    roller = [1/a_i,  (1/a_i)*tan(phi_i)];

    % Rotación de la llanta respecto a {B} (2x2)
    R = [ cos(theta_i)  sin(theta_i);
         -sin(theta_i)  cos(theta_i)];

    % Jacobiano del punto de contacto (2x3)
    J = [1  0  -dy_i;
         0  1   dx_i];

    % Fila cinemática resultante (1x3)
    row = simplify(roller * R * J);
end