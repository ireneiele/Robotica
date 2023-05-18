function [XYd, XYddot] = CartesianPlanner(pi, pf, ti, tf, t)

s = length(pi);

if t<ti
    XYd=pi;
    XYddot=zeros(s,1);
    
elseif t<=tf
    A=[ti^5 ti^4 ti^3 ti^2 ti 1;
       tf^5 tf^4 tf^3 tf^2 tf 1;
       5*ti^4 4*ti^3 3*ti^2 2*ti 1 0;
       5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
       20*ti^3 12*ti^2 6*ti 2 0 0;
       20*tf^3 12*tf^2 6*tf 2 0 0];
    
    b=[0;norm(pi-pf);0;0;0;0];
    x=inv(A)*b;

    s=x(1)*t^5+x(2)*t^4+x(3)*t^3+x(4)*t^2+x(5)*t+x(6);
    sdot=5*x(1)*t^4+4*x(2)*t^3+3*x(3)*t^2+2*x(4)*t+x(5);

    XYd=pi+(s/norm(pi-pf))*(pf-pi);
    XYddot=(sdot/norm(pi-pf))*(pf-pi);
else
    XYd=pf;
    XYddot=zeros(s,1);
end
end

