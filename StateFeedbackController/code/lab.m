%% Initializaion
clc;
clear all;
close all;
m = 0.5; %kg
M = 5; %kg
L = 1; %m
g = 9.8; %m/s^2

%% State space
% the x is theta; theta_dot; p; p_dot
A = [0, 1, 0, 0;
    g*((m+M)/(M*L)), 0, 0, 0;
    0, 0, 0, 1;
    -(m*g/M), 0, 0, 0];

B = [0;
    -1/(M*L);
    0;
    1/M];

[T_A, V_A] = eig(A);
% not stable
%%  State feedback Controller
CI = ctrb(A,B);
rank(CI)
% all controllable

eign_F = [-1+1j; -1-1j; -2+2j; -2-2j];

F = -acker(A, B, eign_F);

C_eyes = eye(4);
D_zeros = zeros(4,1);

Init_cond = [0.1; 0; 0.1; 0];


%% Observer
C = [1, 0, 0, 0;
    0, 0, 1, 0];
D = zeros(2, 1);
O = obsv(A, C);
rank(O)
eign_K = [-100+1j; -100-1j; -200+2j; -200-2j];



K1 = place(A.', C.', eign_K).';
K2 = place(A.', C.', eign_F).';

K = K2;

A_est = [A - K*C];
B_est = [B, K];
% all obervable
%% Plot
close all;
f1=figure(1);
sim('lab_SF');
set(f1,'position',[1 305 672 500]);
subplot(321);
h1 = plot(tout,x( :,1));title('angle');
grid;
subplot(323);
plot(tout,x( :,2));title('d¨¦riv¨¦e angle');
grid;
subplot(322);plot(tout,x( :,3));title('position');
grid;
subplot(324);plot(tout,x( :,4));title('d¨¦riv¨¦e position');
grid;
subplot(325);
h2 = plot(tout,u,'k');title('la commande');
grid;

hh = legend([h1;h2],'x(t)','u(t)',...
    'location',[0.6,0.1,0.3,0.3]);
hold on;

%% 
close all;
f2=figure(2);
sim('lab_SF');
set(f2,'position',[1 305 672 500]);
subplot(321);
h1 = plot(tout,x( :,1));
hold on;
h2 = plot(tout,xi( :,1),'r');
title('angle');

grid;
subplot(323);
plot(tout,x( :,2));
hold on;
plot(tout,xi( :,2),'r');
title('d¨¦riv¨¦e angle');

grid;
subplot(322);plot(tout,x( :,3));
hold on;
plot(tout,xi( :,3),'r');
title('position');

grid;
subplot(324);plot(tout,x( :,4));
hold on;
plot(tout,xi( :,4),'r');
title('d¨¦riv¨¦e position');

grid;
subplot(325);
h3 = plot(tout,u,'k');title('la commande');
grid;

hh = legend([h1;h2;h3],'x(t)','xi(t)','u(t)',...
    'location',[0.6,0.1,0.3,0.3]);
hold on;