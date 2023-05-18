function [pd, pdot] = CartesianPlanner_Progetto(pB,pC,ro,deg,Cen,pdot_prec,tB,tC,t)       
    if t <= tB
        pd = pB;
        pdot = zeros(3,1);
        
    elseif t<=tC
        % La funzione che traccia l'arco di circonferenza sta sotto va solo implementato coi nomi correttA
        R = [0, 1, 0;
             1,0,0;
             0,0,-1];
        
  A=[tB^5 tB^4 tB^3 tB^2 tB 1;
       tC^5 tC^4 tC^3 tC^2 tC 1;
       5*tB^4 4*tB^3 3*tB^2 2*tB 1 0;
       5*tC^4 4*tC^3 3*tC^2 2*tC 1 0;
       20*tB^3 12*tB^2 6*tB 2 0 0;
       20*tC^3 12*tC^2 6*tC 2 0 0];

    b=[0;deg*ro;norm(pdot_prec);0;0;0];
    x=A\b;

    s=x(1)*t^5+x(2)*t^4+x(3)*t^3+x(4)*t^2+x(5)*t+x(6);
    sdot=5*x(1)*t^4+4*x(2)*t^3+3*x(3)*t^2+2*x(4)*t+x(5);

    
    pd=Cen+(R*[ro*cos(s/ro);ro*sin(s/ro);0])';
    pdot=R*[-sdot*sin(s/ro);sdot*cos(s/ro);0];
      
    else
        pd = pC;
        pdot = zeros(3,1);    
  
    end
end

