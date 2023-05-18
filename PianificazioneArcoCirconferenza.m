function [pd, pd_dot,pf]=PianificazioneArcoCirconferenza(p_i,teta,C,z,ti,tf,t)
r=norm(p_i-C);
%teta=atan2(norm(cross(pi-C,pf-C)),dot(pi-C,pf-C));

xc=(p_i'-C)/r;
yc=cross(z,xc);
R=[xc;yc;z'];

sf=teta*r;
A=[ti^5 ti^4 ti^3 ti^2 ti 1;
    tf^5 tf^4 tf^3 tf^2 tf 1;
    5*ti^4 4*ti^3 3*ti^2 2*ti 1 0;
    5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
    20*ti^3 12*ti^2 6*ti 2 0 0;
    20*tf^3 12*tf^2 6*tf 2 0 0];
b=[0;sf;0;0;0;0];
x=inv(A)*b;
s=x(1)*t^5+x(2)*t^4+x(3)*t^3+x(4)*t^2+x(5)*t+x(6);
sdot=5*x(1)*t^4+4*x(2)*t^3+3*x(3)*t^2+2*x(4)*t+x(5);

pd=C+R*[r*cos(s/r);r*sin(s/r);0];
pd_dot=R*[-sdot*sin(s/r);sdot*cos(s/r);0];
pf=C+R*[r*cos(teta);r*sin(teta);0];
end