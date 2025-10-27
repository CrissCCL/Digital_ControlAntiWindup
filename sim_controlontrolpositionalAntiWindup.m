clear all; close all; clc

%% Discretization of First-Order Identified Model (Non-Parametric)
Ts = 0.1;
gp  = tf(20,[50 1]);           % First-order plant
gpd = c2d(gp,Ts,'zoh');        % Discrete plant

%% PI controller parameters 
Kp = 0.8; % propotional gain 
Ti = 9; % integral time
%% Closed-Loop Simulation (Direct difference equation)
[num,den] = tfdata(gpd,'v');
t= 0:Ts:60;
M=20;
Ref = M*ones(1,length(t));

% states
y = 0; 
y1 = 0;
I = 0;
u = 0;

for k=1:length(t)
    % ---- PLANT ----
    y(k) = num(2)*u - den(2)*y1;

    % ---- FEEDBACK WITH ANTIWINDUP ----
    e = Ref(k) - y(k);

    if (u>=100 && e>0) || (u<=0 && e<0)
        I = I;
    else
        I = I + Ts*e;
    end
    u = Kp*e + Kp/Ti*I;

    % ---- SATURATION ---- 
    if u > 100 
        u = 100; 
    end 
    if u < 0 
        u = 0; 
    end
    % ---- UPDATE STATES ----
    y1=y(k);
    Usim(k)=u;
end

%% PLOTS
subplot(2,1,1)
plot(t,y,'+',t,Ref,'--','MarkerSize',4)
xlabel('Time [s]'), ylabel('Response')
legend('Simulation','Reference')

subplot(2,1,2)
plot(t,Usim,'+','MarkerSize',4)
xlabel('Time [s]'), ylabel('Control signal')
