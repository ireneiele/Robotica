%% MASSIMIZZARE MISURA MANIPOLABILITA' mantenendo ferma la posizione dell'organo terminale
clear all
close all
clc

%% Definizione arbitraria caratteristiche manipolatore
%definisco lunghezze link che devono essere coerenti con quelle usate nella
%in funzionale manipolabilità
a1=0.18;
a2=0.15;
a3=0.1;

a=[a1, a2, a3];
lmax = sum(a);

%% Definizione configurazione iniziale in SG (e quindi viene applicata la cinematica diretta per individuare la posa iniziale del manipolatore)
qd1_i=pi/4;
qd2_i=pi/4;
qd3_i=-pi/6;

qd_i=[qd1_i, qd2_i, qd3_i];
%devo fare due simulazioni in parallelo per questo uso qd e qd_man che le
%setto in maniera identica
qd(1,:)=qd_i;
qd_man(1,:)=qd_i; % ottimizza la manipolabilità (uso funzionale)
%calcolo lo jacobiano: do in ingresso configurazioni e lunghezze link
J = Jacobian_3dof(qd(1,:), a);

%estraggo le prime due righe, non ho contributi lungo z perchè manipolatori
%planari
J = J(1:2,:);
%calcolo funzionale nello stesso modo solo all'inizio poi le calcolerò in
%maniera diversa
W(1,1) = sqrt(det(J*J')); % calcolo il funzionale, dove J=2X3 J'=3x2 quindi il loro prodotto è una matrice quadrata 3x3. 
% il funzionale w è uno scalare
W_fun(1,1) = sqrt(det(J*J'));
% w e w_fun inizialmente sono uguali, dalla 2 iterazione cambiano dipendendo
% dalla specifica posizione

T01_i = DH_computation(0, a(1), 0, qd1_i);
T12_i = DH_computation(0, a(2), 0, qd2_i);
T23_i = DH_computation(0, a(3), 0, qd3_i);

T03_i = T01_i*T12_i*T23_i;
%Con la cinematica diretta calcolo configurazione iniziale
p_i = DirectKinematics(T03_i);


%% Definizione configurazione finale in SG (e quindi viene applicata la cinematica diretta per individuare la posa finale del manipolatore)
% mi serve per prelevare un punto finale, ma alla fine della simulazione
% non la raggiungo. QUESTA MI SERVE SOLO PER CALCOLARE LA POSIZIONE FINALE
qd1_f=-pi/4;
qd2_f=0;
qd3_f=pi/3;

qd_f=[qd1_f, qd2_f, qd3_f];

T01_f = DH_computation(0, a(1), 0, qd1_f);
T12_f = DH_computation(0, a(2), 0, qd2_f);
T23_f = DH_computation(0, a(3), 0, qd3_f);

T03_f = T01_f*T12_f*T23_f;

p_f = DirectKinematics(T03_f);

% finita la simulazione NON arriverò a questa config, questa mi serve solo
% per calcolare la posizione finale. arrivo alla pos finale in maniera
% iterativa 
%% Vengono assegnati i valori iniziali delle variabili contenenti posizioni e velocità iniziali in SO
% SO
XYd(1,:)=p_i;
XYddot(1,:)=[0, 0, 0]; %vel iniziale 
%posizione organo terminale sia simulazione con il funzionale che senza
%funzionale
XYe(1,:)=p_i;
XYe_man(1,:)=p_i;

%% Definizione dei tempi di simulazione e del passo delta_t
%uso tempo iniziale e finale per la pianificazione
ti=3;
tf=10;
delta_t=0.01;
t=[0:delta_t:tf];

%% Ciclo for per la simulazione
%il loop della simulazione comincia da i=2
%pianifico il nuovo rif nello SO usando cartesian planner generanso pos e
%vel desiderat nello SO che saranno uguali per entrambi
%generiamo velocità e posizione desiderata nello SO

for i=2:length(t)
    [XYd(i,:), XYddot(i,:)] = CartesianPlanner(p_i, p_f, ti, tf, t(i));
    
    % Calcolo delle velocità attuali nello SG desiderate in uscita
    % dall'algoritmo di inversione
    [qddot(i,:), err(i,:)]=InverseKinematics(qd(i-1,:),a,XYd(i,:),XYddot(i,:),5,"i");
    %qui sotto compare una nuova funzione: compare il numero 10= ka (quello che sta nel proiettore 
    % del nullo) che sarà un
    %altro guadagno (peso che devo dare alla funzione obiettivo secondaria:
    %se la metto a zero non massimizzo manipolabilità)
    %In uscità ho le velocità desiderate nello spazio dei giunti
    [qddot_man(i,:), err_man(i,:)]=InverseKinematicsManipolabilita(qd_man(i-1,:),a,XYd(i,:),XYddot(i,:),5,"i",10);
    %%INTEGRAZIONE CON METODO DI EULERO
    % Integrazione numerica delle velocità nello SG per ottenere le
    % posizione desiderate SG attuali
    qd(i,:) = qd(i-1,:) + qddot(i,:) * delta_t;
    qd_man(i,:) = qd_man(i-1,:) + qddot_man(i,:) * delta_t;
    
    % Calcolo cinematica della simulazione senza funzionale
    T01 = DH_computation(0, a(1), 0, qd(i,1));
    T12 = DH_computation(0, a(2), 0, qd(i,2));
    T23 = DH_computation(0, a(3), 0, qd(i,3));
    T02 = T01*T12;
    T03 = T01*T12*T23;
    XY1(i,:) = DirectKinematics(T01);
    XY2(i,:) = DirectKinematics(T02);
    XYe(i,:) = DirectKinematics(T03);
    
    J = Jacobian_3dof(qd(i,:), a);
    J = J(1:2,:);
    W(i,1) = sqrt(det(J*J'));
    
    % Calcolo cinematica della simulazione con funzionale
    T01 = DH_computation(0, a(1), 0, qd_man(i,1));
    T12 = DH_computation(0, a(2), 0, qd_man(i,2));
    T23 = DH_computation(0, a(3), 0, qd_man(i,3));
    T02 = T01*T12;
    T03 = T01*T12*T23;
    XY1_man(i,:) = DirectKinematics(T01);
    XY2_man(i,:) = DirectKinematics(T02);
    XYe_man(i,:) = DirectKinematics(T03);
    %Calcolo effetto del funzionale
    J_man = Jacobian_3dof(qd_man(i,:), a);
    J_man = J_man(1:2,:);
    W_fun(i,1) = sqrt(det(J_man*J_man'));
end

%%
close all
f2 = figure(2);
f2.OuterPosition = [115,132,1200,712];
f2.Color = [1 1 1];

for i = 1:10:length(t)
    
    sgt = sgtitle('3 DoF Manipulator','Color','black');
    sgt.FontSize = 20;
    sgt.FontWeight = 'bold';
    sgt.Interpreter = 'latex';
    
    subplot(221)
    plot([0],[0],'.k','MarkerSize',20)
    hold on
    plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)
    plot([0],[0],'.k','MarkerSize',20)
    plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
    plot([XY1(i,1)],[XY1(i,2)],'.k','MarkerSize',20)
    plot([XY2(i,1) XYe(i,1)],[XY2(i,2) XYe(i,2)],'-g','Linewidth',4)
    plot([XY2(i,1)],[XY2(i,2)],'.k','MarkerSize',20)
    plot(XYd(1:i,1),XYd(1:i,2),'--r','LineWidth',3)
    plot(XYe(1:i,1),XYe(1:i,2),'-b','LineWidth',1.5)
    plot(p_f(1),p_f(2),'k*','MarkerSize',20)
    text(p_f(1)+0.05,p_f(2),'$p_f$','Interpreter','latex','FontSize',16)
    plot(p_i(1),p_i(2),'k*','MarkerSize',20)
    text(p_i(1)+0.05,p_i(2),'$p_i$','Interpreter','latex','FontSize',16)
    
    xlim([-lmax lmax])
    ylim([-lmax lmax])
    
    xlabel('x [m]','Interpreter','latex')
    ylabel('y [m]','Interpreter','latex')
    
    set(gca,'FontSize',16)
    hold off
    grid on
    title("SENZA FUNZIONALE",'Interpreter','latex')
    
    subplot(222)
    plot([0],[0],'.k','MarkerSize',20)
    hold on
    plot([0 XY1_man(i,1)],[0 XY1_man(i,2)],'-r','Linewidth',4)
    plot([0],[0],'.k','MarkerSize',20)
    plot([XY1_man(i,1) XY2_man(i,1)],[XY1_man(i,2) XY2_man(i,2)],'-b','Linewidth',4)
    plot([XY1_man(i,1)],[XY1_man(i,2)],'.k','MarkerSize',20)
    plot([XY2_man(i,1) XYe_man(i,1)],[XY2_man(i,2) XYe_man(i,2)],'-g','Linewidth',4)
    plot([XY2_man(i,1)],[XY2_man(i,2)],'.k','MarkerSize',20)
    plot(XYd(1:i,1),XYd(1:i,2),'--r','LineWidth',3)
    plot(XYe_man(1:i,1),XYe_man(1:i,2),'-b','LineWidth',1.5)
    plot(p_f(1),p_f(2),'k*','MarkerSize',20)
    text(p_f(1)+0.05,p_f(2),'$p_f$','Interpreter','latex','FontSize',16)
    plot(p_i(1),p_i(2),'k*','MarkerSize',20)
    text(p_i(1)+0.05,p_i(2),'$p_i$','Interpreter','latex','FontSize',16)
    
    xlim([-lmax lmax])
    ylim([-lmax lmax])
    
    xlabel('x [m]','Interpreter','latex')
    ylabel('y [m]','Interpreter','latex')
    
    set(gca,'FontSize',16)
    hold off
    grid on
    title("CON FUNZIONALE",'Interpreter','latex')
    
    
    subplot(2,2,[3:4])
    plot(t(1:i), W(1:i), 'LineWidth', 3)
    hold on
    plot(t(1:i), W_fun(1:i), 'LineWidth', 3)
    xlim([0 t(end)])
    ylim([0.02 0.06])
    xlabel("Time [s]",'FontSize',18,'Interpreter','latex')
    ylabel("$\omega [m^3]$",'FontSize',18,'Interpreter','latex')
    set(gca,'FontSize',16)
    lg = legend("SENZA FUNZIONALE", "CON FUNZIONALE");
    lg.FontSize = 18;
    lg.Interpreter = 'latex';
    lg.Location = 'bestoutside';
    
    pause(0.01)
    
end

% tutte e due le simulaziono hanno soddisfatto il task, ma la configurazione finale
% dei 2 robot è diversa (grafici in alto). Nel grafico sotto viene mostrato
% il valore del funzionale (è il volume dell'ellissoide m^3 perciò ho
% quell'unita di misura).senza funzionale andamento costante, con
% funzionale ha un andamento prima lineare. Questi sono grafici
% qualitativi. Per mostrare in maniera quantitativa l'effetto del
% funzionale calcolo la media della linea blu e della linea rossa, posso
% dire che in media ho un aumento di un certo valore, oppure posso fare la
% differenza punto-punto tra funzionale e non funzionale
 
% osserviamo la simulazione per i primi 3 secondi:
% il robot a sx non si muove, quello a dx si inizia già a muoversi
% per massimizzare la manipolabilità

%Se metto come guadagno del funzionale ka (invece di 10)100: cosa succede?
% Per i primi 3 secondi la pendenza aumenta: ci metto meno tempo ad avere
% una manipolabilità più alta. Quindi a parità di posizione da mantenere 
% dell'organo terminale  il robot si riconfigura per avere una manipolabilità 
% più alta. durante il percorso manyterrà un configurazione che massimizz
% ala manipolabilità

% se metto 1000: ottengo una specie di gradino. Maggiore è il k maggiore
%sono le velocità dei moti interni.
% k Non posso spingerlo all'infinito altrimenti ho dei set di velocità troppo
%elevati.

%SE DOVESSI FARE STESSA COSA SU ROBOT A 4 gdl, dove devo modificare il codice?
% 1)Devo aggiungere lunghezza link, 2)devo fare uno step in più sulla
% cinematica diretta. 3)Codice Funzionemanipolabilità (come calcolo il funzionale) anche cambia, 4)anche il calcolo
% dello jacobiano
% La pianificazione di traiettoria(cartesianPlanner) non è necessario
%cambiarla perchè è indipendente dal robot specifico.

%Devi verificare che l'errore non aumenti: cioè effettivamente il manipolatore
%deve seguire traiettoria. 
%1. inseguire la traiettoria 
%2.verificare che l'errore nell'inversione nomn aumenti troppo

%NB. per calcolare la distanza da un ostacolo non devo prendere l'end
%effector ma un altro giunto. Lascio in forma simbolica  anche la distanza
%dall'ostacolo (nel caso risenti per progetto lezione 20/03 parte 1 min 55)
