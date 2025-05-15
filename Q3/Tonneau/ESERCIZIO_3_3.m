clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                            ESERCIZIO 3.3                                %
%                                                                         %
%                         MANOVRA DI TONNEAU                              %
%          Cinematica dell'evoluzione di tonneau in Matlab                %
%                                                                         %
%                                             NOME: Vincenzo              %
%                                             COGNOME: Romano             %
%                                             N° MATRICOLA: N53/000916    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Tempo finale di simulazione
t_fin = 105.0;     %tempo finale pari a 105 secondi

% Condizioni iniziali
psi0 = 0;  theta0 = 0;  phi0 = 0;          % Angoli di Eulero
X0=0;      Y0=0;        Z0 = 0;            % Posizione iniziale

% Occorre assegnare i seguenti valori:
% p = p(t); q = q(t); r = 0.
% u = u(t); v = 0; w = 0.

% Funzioni anonime
% Storie temporali di p, q, r
p_max = convangvel(4.0,'deg/s','rad/s');  % converte deg/s in rad/s
q_max = convangvel(2.0,'deg/s','rad/s');  % converte deg/s in rad/s
r_max = 0;

vettoreTempo = [0, t_fin/30, t_fin/10, t_fin/5, 0.7*t_fin, 0.9*t_fin, t_fin];
vettoreP = [0, p_max/40, p_max*3/4,  p_max,     p_max,         0,     0];
vettoreQ = [0, q_max/40, q_max*3/4,  q_max,     q_max,         0,     0];
vettoreR = [0,        0,         0,      0,         0,         0,     0];


p = @(t) ... %funzione anonima della variabile muta temporale t
interp1( ... 
   [0, t_fin/30, t_fin/10, t_fin/5, 0.7*t_fin, 0.9*t_fin, t_fin], ...
   [0, p_max/40, p_max*3/4,  p_max,     p_max,         0,     0], ...
   t, 'pchip' ... %'pchip' Piecewise Cubic Hermite Interpolating Polynomial
);                %ovvero tipologia di interpolante Hermitiana

q = @(t) ... 
interp1( ...
   [0, t_fin/30,  t_fin/10, t_fin/5, 0.8*t_fin, 0.9*t_fin, t_fin], ...
   [0, q_max/40, q_max*3/4,   q_max,     q_max,         0,     0], ...
   t, 'pchip' ...
);

r = @(t) 0*t;
        
% Storie temporali di u, v, w
u0 = convvel(380.0,'km/h','m/s'); % converte km/h in m/s
v0 = convvel( 0.0,'km/h','m/s');  % converte km/h in m/s
w0 = convvel( 0.0,'km/h','m/s');  % converte km/h in m/s

vettoreTempo = [0, t_fin/30, t_fin/10, t_fin/5, 0.7*t_fin, 0.9*t_fin, t_fin];
vettoreU = [u0,   0.8*u0,   0.7*u0,      u0,    1.1*u0,   1.07*u0,    u0];
vettoreV = [0,        0,         0,      0,         0,         0,     0];
vettoreW = [0,        0,         0,      0,         0,         0,     0];

u = @(t) interp1( ...
   [ 0, t_fin/30, t_fin/10, t_fin/5, 0.8*t_fin, 0.9*t_fin, t_fin], ...
   [u0,   0.8*u0,   0.7*u0,      u0,    1.1*u0,   1.07*u0,    u0], ...
   t, 'pchip' ...
);

% Assumiamo v(t) = w(t) = 0
v = @(t) 0*t;
w = @(t) 0*t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Mesh
N = 120; %numero di punti
t = linspace (0, t_fin, N); %punti del mesh uniforme

% Plot p(t), q(t), r(t)
figure (1)
plot(t, p(t)*180/pi, '.-b', t, q(t)*180/pi, '.-r', t, r(t)*180/pi, '.-g');
hold on;
grid on;
ylabel ('(deg/s)');
xlabel ('\it t (s)'); %\it per avere la scritta in corsivo
legend ('p(t)','q(t)', 'r(t)'); 
title ('Componenti p, q ed r della velocità angolare');
axis ([0 t_fin -0.2 p_max*(180/pi)+0.2]);

% Plot u(t)
figure (2)
plot(t, u(t), '.-b');
hold on;
grid on;
ylabel ('(m/s)');
xlabel ('\it t (s)'); %\it per avere la scritta in corsivo
legend({'u(t)'},'Location','southeast');
title ('Componente u della velocità del baricentro');
axis ([0 t_fin 70 120]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Equazioni cinematiche - vedi le equazioni (3.67)
dQuatdt = @(t, Q) ... %vettore derivate dei quadernioni rispetto dt
0.5*[    0, -p(t), -q(t), -r(t);
      p(t),     0,  r(t), -q(t);
      q(t), -r(t),     0,  p(t);
      r(t),  q(t), -p(t),     0] * Q; %Q = vettore dei quadernioni

% Soluzione delle equazioni di evoluzioni dei quadernioni  
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9*ones(1,4));

% Condizioni iniziali quadernioni
Q0 = angle2quat(psi0, theta0, phi0); %si veda l'equazione (3.61)
     % La funzione angle2quat converte gli "angoli (angle)" in (to)
     % "quaternioni (quat)"
    
% Integrazione Runge-Kutta
[vTime, vQuat] = ode45(dQuatdt, [0 t_fin], Q0, options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3)
% Quaternion components time histories !see Fig. 3.13a
subplot 121 %subplot di 1 riga, 2 colonne nella posizione 1 
plot( ...
vTime, vQuat(:,1), '-b',  ... %vettore q0
vTime, vQuat(:,2), '--r', ... %vettore qx
vTime, vQuat(:,3), '-.g', ... %vettore qy
vTime, vQuat(:,4), ':k'   ... %vettore qz
);
grid on;
xlabel('\it t (s)'); 
legend('q_0(t)','q_x(t)','q_y(t)','q_z(t)');
title('Componenti dei quadernioni')
axis ([0 inf -1 1]);

% Storia degli angoli di Eulero
subplot 122 %subplot di 1 riga, 2 colonne nella posizione 2 
[vpsi, vtheta, vphi] = quat2angle(vQuat); %funzione per passare da "quat" 
                                          %"to" "angle".
plot( ...
vTime, convang(  vpsi,'rad','deg'), '-b', ...
vTime, convang(vtheta,'rad','deg'), ':r', ...
vTime, convang(  vphi,'rad','deg'), '--g' );
grid on;
xlabel('\it t (s)'); 
ylabel('(deg)');
legend('\psi(t)','\theta(t)','\phi(t)');
title('Angoli di Eulero');
axis ([0 inf -190 190]);

figure(4)
% Componenti velocità angolari in assi body
subplot 211 %subplot di 2 riga, 1 colonne nella posizione 1 
plot( ...
vTime, convangvel(p(vTime),'rad/s','deg/s'), '-b', ...
vTime, convangvel(q(vTime),'rad/s','deg/s'), ':r', ...
t, r(t)*180/pi, '-g');
grid on;
xlabel('\it t (s)'); 
ylabel('(deg/s)');
legend('p(t)','q(t)','r(t)');
title('Componenti velocità in assi body');
axis ([0 t_fin -0.2 p_max*(180/pi)+0.2]);

% Componenti delle velocità lineari in assi body 
subplot 212 %subplot di 2 riga, 1 colonne nella posizione 2 
plot(vTime, u(vTime), '-b');
grid on;
xlabel('\it t (s)'); 
ylabel('(m/s)');
legend('u(t)');
title('Compoenente velocità lungo l''asse x in asse body');
axis ([0 t_fin 70 120]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Funzione di interpolazione temporale della storia dei quaternioni
Quat = @(t) ...                    %funzione anonima
[interp1(vTime,vQuat(:,1),t), ...  %funzione q0(t)
 interp1(vTime,vQuat(:,2),t), ...  %funzione qx(t)
 interp1(vTime,vQuat(:,3),t), ...  %funzione qy(t)
 interp1(vTime,vQuat(:,4),t)];     %funzione qz(t)

% Matrice di trasformazione da assi Earth ad assi body: T_BE = [T_EB]^T
T_BE = @(Q) quat2dcm(Q);           %Da "quat" "to" "dcm (direction cosin matrix)

% Definzione delle navigation equations - si veda eq. (3.25)
dPosEdt = @(t,PosE) ... %derivate posizione rispetto dt nel riferimento Earth
transpose(quat2dcm(Quat(t)))*[u(t);v(t);w(t)]; 
         %transpose(quat2dcm(Quat(t))) = [T_EB]^T

%Soluzione delle navigation equations
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3*ones(3,1));

% Posizione iniziale nel riferimento Earth
PosE0 = [X0;Y0;Z0];

% Integrazione mediante Runge-Kutta
[vTime2, vPosE] = ode45(dPosEdt, vTime, PosE0, options);

% Vettori posizione x_E, y_E, z_E  
vXe = vPosE(:,1);
vYe = vPosE(:,2);
vZe = vPosE(:,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot delle coordinate del baricentro espresse nel riferimento earth
figure(5)
plot(vTime, vXe, '-b',...
     vTime, vYe, '-.r', ...
     vTime, vZe, '--g');
grid on;
xlabel('\it t (s)'); 
ylabel('(m)');
legend({'x_{G,E}(t)','y_{G,E}(t)','z_{G,E}(t)'}, 'Location', 'northwest');
title('Coordiante del baricentro ngli assi Earth');
axis ([0 t_fin -2000 10000]);

% Plot body and trajectory !see Fig. 3.12
h_fig6 = figure(6);
title('Traiettoria della manovra di tonneau a destra');
grid on;
%theView = [70 20];
theView = [1,1,0.5];
plotTrajectoryAndBody(h_fig6,vXe,vYe,vZe,vQuat,0.003,25,theView);
          % Fattore di scala 0.003, ossia ingrandisce l'aereo di 3000 volte
          % 25 = disegna l'aereo ogni 25 istanti
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

