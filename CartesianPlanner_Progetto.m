function [pd, pdot] = CartesianPlanner_Progetto(pA,pB,pC,ro,deg,Cen,tA,tB,tC,t)       
    if t <= tA
        pd = pA;
        pdot = zeros(3,1);

    
    else if tA<t <= tB
        % Questa parte serve a rappresentare la traslazione da A a B
        A = [tA^3, tA^2, tA, 1;
             tB^3, tB^2, tB, 1;
             3*tA^2, 2*tA, 1, 0;
             3*tB^2, 2*tB, 1, 0];

        b = [norm(pA),norm(pB-pA),0,0];

        x = inv(A)*b';
 
        s = x(1)*t^3+x(2)*t^2+x(2)*t+x(3);
        sdot = 3*x(1)*t^2+2*x(2)*t+x(2);
        
        pd=pA+(s/norm(pB-pA))*(pB-pA);
        pdot=(sdot/norm(pA-pB))*(pB-pA);
        
    else if tB<t <= tC
        % La funzione che traccia l'arco di circonferenza sta sotto va solo implementato coi nomi corretti
        R = [0, 1, 0;
             1,0,0;
             0,0,-1];
        
        A = [tB^3, tB^2, tB, 1;
             tC^3, tC^2, tC, 1;
             3*tB^2, 2*tB, 1, 0;
             3*tC^2, 2*tC, 1, 0]

        b = [norm(pB),ro*deg,norm(pdot),0];

                
        x = inv(A)*b';
 
        s = x(1)*t^3+x(2)*t^2+x(2)*t+x(3);
        sdot = 3*x(1)*t^2+2*x(2)*t+x(2);
        
        pd = Cen+ R*[ro*cos(s/ro);ro*sin(s/ro);0];
        pdot = R*[-sdot*sin(s/ro);sdot*cos(s/ro);0];

        
    else if t > tC
        pd = pC;
        pdot = zeros(3,1);    
  
    end
    end
    end
    end
