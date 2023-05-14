function J = Jacobian_4dof(q,a)

    T01 = DH_computation(0, a(1), 0, q(1));
    T12 = DH_computation(0, a(2), 0, q(2));
    T23 = DH_computation(0, a(3), 0, q(3));
    T34 = DH_computation(0, a(4), 0, q(4));

% 
%     T01 = DH_computation(d(1), a(1), sym (pi/2), q(1));
%     T12 = DH_computation(d(2), a(2), sym (-pi/2), q(2));
%     T23 = DH_computation(d(3), a(3),sym (pi/2), q(3));
%     T34 = DH_computation(d(4), a(4), 0, q(4));

    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
% Definizione elementi per il calcolo dello Jacobiano
    z0= [0 0 1]'; 
    z1= T01(1:3,3); 
    z2= T02(1:3,3);
    z3= T03(1:3,3);
    
    p0=[0 0 0]';
    p1=T01(1:3,4);
    p2=T02(1:3,4);
    p3= T03(1:3,4);
    p=T04(1:3,4);

    J=[cross(z0,(p-p0)) cross(z1,(p-p1)) cross(z2,(p-p2)) cross(z3,(p-p3))]; % parte posizionale Matrice Jacobiana

end