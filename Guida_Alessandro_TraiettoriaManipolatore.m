clear all
close all
clc

%% Definizione Manipolatore 

% Definizione parametri manipolatore

a1=0.7;
a2=0.74;
a3=0.3;
a4=0.65; 

d1=0;
d2=0.35;
d3=0;
d4=1.4;

alpha1=-pi/2;
alpha2=pi/2;
alpha3=pi/2;
alpha4=0;

a=[a1,a2,a3,a4];
d=[d1,d2,d3,d4];
alpha=[alpha1, alpha2, alpha3, alpha4];
lmax = sum(a)+sum(d); % Per fare i plot utilizzo dei limiti direttamente su quelli del manipolatore

%% Plot dei possibili punti raggiungibili dal manipolatore

count = 1;

f = figure(1);
f.OuterPosition = [115,132,940,712]; % Posizione figura

% tic-toc per definire tutti i possibili punti raggiungibili dal
% manipolatore nel suo spazio di lavoro attraverso le rototraslazioni

tic

% Per ognuno definisco variabili di giunto tra ipotetici punti
% raggiungibili osservarvo il reale e dove non vada a causare problemi

% I punti di definizione nel linspace risultano essere presi direttamente
% sul manipolatore in maniera da osservare come questo può ruotare ed
% evitare problemi con il collegamento dei motori sopratutto

for q1 = linspace(0,2*pi , 15)
    for q2 = linspace(-pi/2,pi, 15)
        for q3 = linspace(-pi/10,pi/4, 15)
            for q4 = linspace(0, 2*pi,15)

         T01 = DH_computation(d1, a1, alpha1,q1) ;
    T12 = DH_computation(d2, a2, alpha2, q2);
    Ti = DH_computation(0,0,0,pi/2);
    T23 = DH_computation(d3, a3, alpha3, q3);
    T34 = DH_computation(d4, a4, alpha4, q4);
        
    T02 = T01*T12;
    T0i= T02*Ti;
    T03 =T0i*T23;
    T04 = T03*T34;
       
% Cinematica Diretta
        xy1 = DirectKinematics(T01);
        xy2 = DirectKinematics(T02);
        xyi = DirectKinematics(T0i);
        xy3 = DirectKinematics(T03);
        xy4 = DirectKinematics(T04);
        
        % Posizioni raggiungili nello spazio dal manipolatore
        P(count,:) = xy4;
        
        count = count + 1;
        
        % Plot delle posizioni raggiungibili

        plot3(P(:,1), P(:,2), P(:,3),'.','MarkerSize',10)
        hold on
        bound = boundary(P(:,1),P(:,2), P(:,3),1);
        plot3(P(bound,1),P(bound,2),P(bound,3),'LineWidth',3)
        
        plot3([0 xy1(1)],[0 xy1(2)], [0 xy1(3)],'r','LineWidth',8,'HandleVisibility','off')
        hold on
        plot3([xy1(1) xy2(1)],[xy1(2) xy2(2)], [xy1(3) xy2(3)],'b','LineWidth',8,'HandleVisibility','off')
        plot3(0,0,0,'k.','MarkerSize',45)
        plot3(xy1(1),xy1(2),xy1(3),'k.','MarkerSize',35)
        plot3(xy2(1),xy2(2),xy2(3),'k.','MarkerSize',35)
        plot3(xyi(1),xyi(2),xyi(3),'k.','MarkerSize',35)
        plot3(xy3(1),xy3(2),xy3(3),'k.','MarkerSize',35)
        plot3(xy4(1),xy4(2),xy4(3),'k.','MarkerSize',35)
        
        hold off
        
        grid
        set(gca,'FontSize',18)
        
        xlabel('x [m]','Interpreter','latex','FontSize',24)
        ylabel('y [m]','Interpreter','latex','FontSize',24)
        zlabel ('z [m]', 'Interpreter','latex','FontSize',24)
        
        axis equal
        
            end
        end
    end
end
toc