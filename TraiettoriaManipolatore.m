clear all
close all
clc

%% Definizione Manipolatore 

% Definizione parametri manipolatore
clc 
clear 
close all

a1=0.75;
a2=0.65;
a3=0.2;
a4=0; 

d1=0.65;
d2=0.42;
d3=0.37;
d4=0.5;

alpha1=pi/2;
alpha2=-pi/2;
alpha3=pi/2;
alpha4=0;

a=[a1,a2,a3,a4];
d=[d1,d2,d3,d4];
alpha=[alpha1, alpha2, alpha3, alpha4];
lmax = sum(a)+sum(d); % Per fare i plot utilizzo dei limiti direttamente su quelli del manipolatore

%% Plot dei possibili punti raggiungibili dal manipolatore

i = 1;

% f.OuterPosition = [115,132,940,712]; % Posizione figura

% tic-toc per definire tutti i possibili punti raggiungibili dal
% manipolatore nel suo spazio di lavoro attraverso le rototraslazioni

% Per ognuno definisco variabili di giunto tra ipotetici punti
% raggiungibili osservarvo il reale e dove non vada a causare problemi

% I punti di definizione nel linspace risultano essere presi direttamente
% sul manipolatore in maniera da osservare come questo può ruotare ed
% evitare problemi con il collegamento dei motori sopratutto

for q1 = linspace(0,2*pi , 4)
for q2 = linspace(-pi/4,pi/4, 4)
for q3 = linspace(0,2*pi, 4)
for q4 = linspace(-pi/4, pi*4,4)
    T01 = DH_computation(d1, a1, alpha1,q1) ;
    T12 = DH_computation(d2, a2, alpha2, q2);
    T23 = DH_computation(d3, a3, alpha3, q3);
    T34 = DH_computation(d4, a4, alpha4, q4);
        
    T02 = T01*T12;
    T03 =T02*T23;
    T04 = T03*T34;
       
% Cinematica Diretta
    xy1 = DirectKinematics(T01);
    xy2 = DirectKinematics(T02);
    xy3 = DirectKinematics(T03);
    xy4 = DirectKinematics(T04);

    P(i,:) = xy4;


    i = i+1;
   
end
end
end
end

    % Posizioni raggiungili nello spazio dal manipolatore

    plot3(P(:,1), P(:,2), P(:,3),'.','MarkerSize',10)
    hold on
%   bound = boundary(P(:,1),P(:,2), P(:,3),1);
%   plot3(P(bound,1),P(bound,2),P(bound,3),'LineWidth',3)
    
    plot3([0 xy1(1)],[0 xy1(2)], [0 xy1(3)],'r','LineWidth',8,'HandleVisibility','off')
    hold on
    plot3([xy1(1) xy2(1)],[xy1(2) xy2(2)], [xy1(3) xy2(3)],'b','LineWidth',8,'HandleVisibility','off')
    plot3([xy2(1) xy3(1)],[xy2(2) xy3(2)], [xy2(3) xy3(3)],'g','LineWidth',8,'HandleVisibility','off')
    plot3([xy3(1) xy4(1)],[xy3(2) xy4(2)], [xy3(3) xy4(3)],'y','LineWidth',8,'HandleVisibility','off')


    plot3(0,0,0,'k.','MarkerSize',45)
    plot3(xy1(1),xy1(2),xy1(3),'k.','MarkerSize',25)
    plot3(xy2(1),xy2(2),xy2(3),'k.','MarkerSize',25)
    plot3(xy3(1),xy3(2),xy3(3),'k.','MarkerSize',25)
    
    grid
    set(gca,'FontSize',18)
    
    xlabel('x [m]','Interpreter','latex','FontSize',24)
    ylabel('y [m]','Interpreter','latex','FontSize',24)
    zlabel ('z [m]', 'Interpreter','latex','FontSize',24)
    
    axis equal