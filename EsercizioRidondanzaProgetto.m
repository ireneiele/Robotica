%% MASSIMIZZARE MISURA MANIPOLABILITA' mantenendo ferma la posizione dell'organo terminale
clear all
close all
clc

%% Definizione arbitraria caratteristiche manipolatore
%definisco lunghezze link che devono essere coerenti con quelle usate nella
%in funzionale manipolabilità

a1=0.75;  %cm
a2=0.65;
a3=0.2;
a4=0; 

d1=0.65;
d2=0.42;
d3=0.37;
d4=0.05;

alpha1=pi/2;
alpha2=-pi/2;
alpha3=pi/2;
alpha4=0;

a=[a1,a2,a3,a4];
d=[d1,d2,d3,d4];
alpha=[alpha1, alpha2, alpha3, alpha4];
lmax = sum(a)+sum(d); 

%% Definizione configurazione iniziale in SG
% si applica la cinematica diretta per individuare la posa iniziale del manipolatore
qd1_i=-pi/2;
qd2_i=pi/6;
qd3_i=0;
qd4_i=pi/4;

qd_i=[qd1_i, qd2_i, qd3_i, qd4_i];
%devo fare due simulazioni in parallelo per questo uso qd e qd_man che le
%setto in maniera identica

qd(1,:) = qd_i;
qd_man(1,:)=qd_i; % ottimizza la manipolabilità (uso funzionale)
%calcolo il funzionale: 

T01_i = DH_computation(d(1), a(1), alpha(1), qd1_i);
T12_i = DH_computation(d(2), a(2), alpha(2), qd2_i);
T23_i = DH_computation(d(3), a(3), alpha(3), qd3_i);
T34_i = DH_computation(d(4), a(4), alpha(4), qd4_i);

T04_i = T01_i*T12_i*T23_i*T34_i;
%Con la cinematica diretta calcolo configurazione iniziale
pA = DirectKinematics(T04_i);


%% Definizione configurazione intermedia in SG 
% si applica la cinematica diretta per individuare la posa iniziale del manipolatore
qd1_f=-pi/4;
qd2_f=0;
qd3_f=pi/3;
qd4_f=-pi/2;

qd_f=[qd1_f, qd2_f, qd3_f, qd4_f];

T01_f = DH_computation(d(1), a(1), alpha(1), qd1_f);
T12_f = DH_computation(d(2), a(2), alpha(2), qd2_f);
T23_f = DH_computation(d(3), a(3), alpha(3), qd3_f);
T34_f = DH_computation(d(4), a(4), alpha(4), qd4_f);


T04_f = T01_f*T12_f*T23_f*T34_f;

pB = DirectKinematics(T04_f); 

% finita la simulazione NON arriverò a questa config, questa mi serve solo
% per calcolare la posizione finale. arrivo alla pos finale in maniera
% iterativa 

%% Vengono assegnati i valori iniziali delle variabili contenenti posizioni e velocità iniziali in SO
% SO
XYd(1,:)=pA; %posizione iniziale
XYddot(1,:)=[0, 0, 0]; %velocità iniziale 
%posizione organo terminale sia simulazione con il funzionale che senza
%funzionale

XYe(1,:) = pA;
XYe_man(1,:)=pA;

%% Definizione dei tempi di simulazione e del passo delta_t
%uso tempo iniziale e finale per la pianificazione
tA=3;
tB=10;
tC=20;
delta_t=0.01;
t=[0:delta_t:tC];



%% Ciclo for per la simulazione
%il loop della simulazione comincia da i=2
%pianifico il nuovo rif nello SO usando cartesian planner generanso pos e
%vel desiderat nello SO che saranno uguali per entrambi
%generiamo velocità e posizione desiderata
ro=0.3285;
Cen=[1 2 1.02];
pC=[-1.39663,-0.24671,1.02];
deg=pi/4;
Kgain = 5;
Ka = 10;

for i=2:length(t)
   if t(i)<=tB
        [XYd(i,:), XYddot(i,:)] = CartesianPlanner_Segmento(pA,pB,tA,tB,t(i));
   else
        [XYd(i,:), XYddot(i,:)] = CartesianPlanner_Progetto(pB,pC,ro,deg,Cen,XYddot(i-1,:),tB,tC,t(i));
   end

    % Calcolo delle velocità attuali nello SG desiderate in uscita
    % dall'algoritmo di inversione
 
    %qui sotto compare una nuova funzione: compare il numero 10= ka (quello che sta nel proiettore 
    % del nullo) che sarà un
    %altro guadagno (peso che devo dare alla funzione obiettivo secondaria:
    %se la metto a zero non massimizzo manipolabilità)
    %In uscità ho le velocità desiderate nello spazio dei giunti
    [qddot(i,:), err(i,:)]=InverseKinematicsProgetto(qd(i-1,:),a,d,alpha,XYd(i,:),XYddot(i,:),Kgain,"i");
    [qddot_man(i,:), err_man(i,:)]=InverseKinematicsProgetto_Man(qd_man(i-1,:),a,d,alpha,XYd(i,:),XYddot(i,:),Kgain,"i",Ka);
    %%INTEGRAZIONE CON METODO DI EULERO
    % Integrazione numerica delle velocità nello SG per ottenere le
    % posizione desiderate SG attuali
    qd(i,:) = qd(i-1,:) + qddot(i,:) * delta_t;
    qd_man(i,:) = qd_man(i-1,:) + qddot_man(i,:) * delta_t;


    % Calcolo cinematica della simulazione senza funzionale

    T01 = DH_computation(d(1), a(1), alpha(1), qd(i,1));
    T12 = DH_computation(d(2), a(2), alpha(2), qd(i,2));
    T23 = DH_computation(d(3), a(3), alpha(3), qd(i,3));
    T34 = DH_computation(d(4), a(4), alpha(4), qd(i,4));

    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;

    XY1(i,:) = DirectKinematics(T01);
    XY2(i,:) = DirectKinematics(T02);
    XY3(i,:) = DirectKinematics(T03);
    XYe(i,:) = DirectKinematics(T04);

    
  
    % Calcolo cinematica della simulazione con funzionale
    T01 = DH_computation(d(1), a(1), alpha(1), qd_man(i,1));
    T12 = DH_computation(d(2), a(2), alpha(2), qd_man(i,2));
    T23 = DH_computation(d(3), a(3), alpha(3), qd_man(i,3));
    T34 = DH_computation(d(4), a(4), alpha(4), qd_man(i,4));

    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;

    XY1_man(i,:) = DirectKinematics(T01);
    XY2_man(i,:) = DirectKinematics(T02);
    XY3_man(i,:) = DirectKinematics(T03);
    XYe_man(i,:) = DirectKinematics(T04);


%     %Calcolo effetto del funzionale
%     J_man = Jacobian_3dof(qd_man(i,:), a);
%     J_man = J_man(1:2,:);
%     W_fun(i,1) = sqrt(det(J_man*J_man'));
   
end

%%
close all
f2 = figure(2);
f2.OuterPosition = [115,132,1200,712];
f2.Color = [1 1 1];

for i = 2:10:length(t)    
    sgt = sgtitle('3 DoF Manipulator','Color','black');
    sgt.FontSize = 20;
    sgt.FontWeight = 'bold';
    sgt.Interpreter = 'latex';
    
%     subplot(222)

    plot([0],[0],'.k','MarkerSize',20)
    hold on
    plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)
    plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
    plot([XY1(i,1)],[XY1(i,2)],'.k','MarkerSize',20)
    plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
    plot([XY2(i,1)],[XY2(i,2)],'.k','MarkerSize',20)
    plot([XY3(i,1) XYe(i,1)],[XY3(i,2) XYe(i,2)],'-y','Linewidth',4)
    plot([XY3(i,1)],[XY3(i,2)],'.k','MarkerSize',20)
    
    plot(XYd(1:i,1),XYd(1:i,2),'--r','LineWidth',3)
    plot(XYe(1:i,1),XYe(1:i,2),'-b','LineWidth',1.5)
    plot(pB(1),pB(2),'k*','MarkerSize',20)
    text(pB(1)+0.05,pB(2),'$p_f$','Interpreter','latex','FontSize',16)
    plot(pA(1),pA(2),'k*','MarkerSize',20)
    text(pA(1)+0.05,pA(2),'$p_i$','Interpreter','latex','FontSize',16)



    plot([0],[0],'.k','MarkerSize',20)
    hold on
    plot([0 XY1_man(i,1)],[0 XY1_man(i,2)],'-r','Linewidth',4)
    plot([XY1_man(i,1) XY2_man(i,1)],[XY1_man(i,2) XY2_man(i,2)],'-b','Linewidth',4)
    plot([XY1_man(i,1)],[XY1_man(i,2)],'.k','MarkerSize',20)
    plot([XY2_man(i,1) XY3_man(i,1)],[XY2_man(i,2) XY3_man(i,2)],'-g','Linewidth',4)
    plot([XY2_man(i,1)],[XY2_man(i,2)],'.k','MarkerSize',20)
    plot([XY3_man(i,1) XYe_man(i,1)],[XY3_man(i,2) XYe_man(i,2)],'-y','Linewidth',4)
    plot([XY3_man(i,1)],[XY3_man(i,2)],'.k','MarkerSize',20)
    
    plot(XYd(1:i,1),XYd(1:i,2),'--r','LineWidth',3)
    plot(XYe_man(1:i,1),XYe_man(1:i,2),'-b','LineWidth',1.5)
    plot(pB(1),pB(2),'k*','MarkerSize',20)
    text(pB(1)+0.05,pB(2),'$p_f$','Interpreter','latex','FontSize',16)
    plot(pA(1),pA(2),'k*','MarkerSize',20)
    text(pA(1)+0.05,pA(2),'$p_i$','Interpreter','latex','FontSize',16)
    
    xlim([-lmax lmax])
    ylim([-lmax lmax])
    
    xlabel('x [m]','Interpreter','latex')
    ylabel('y [m]','Interpreter','latex')
    
    set(gca,'FontSize',16)
    hold off
    grid on
    title("CON FUNZIONALE",'Interpreter','latex')
    
    
%     subplot(2,2,[3:4])
%     plot(t(1:i), W(1:i), 'LineWidth', 3)
%     hold on
%     plot(t(1:i), W_fun(1:i), 'LineWidth', 3)
%     xlim([0 t(end)])
%     ylim([0.02 0.06])
%     xlabel("Time [s]",'FontSize',18,'Interpreter','latex')
%     ylabel("$\omega [m^3]$",'FontSize',18,'Interpreter','latex')
%     set(gca,'FontSize',16)
%     lg = legend("SENZA FUNZIONALE", "CON FUNZIONALE");
%     lg.FontSize = 18;
%     lg.Interpreter = 'latex';
%     lg.Location = 'bestoutside';
    
    pause(0.01)
    
end
