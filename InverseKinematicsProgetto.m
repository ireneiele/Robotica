%% Algoritmo di Inversione Cinematica

function [qddot, e] = InverseKinematicsProgetto(q, a, XYd, XYddot, kgain, method, ka)

% Calcolo lo Jacobiano geometrico
J = Jacobian_4dof(q, a);

% Calcolo la posizione attuale
T01 = DH_computation(0, a(1), 0, q(1));
T12 = DH_computation(0, a(2), 0, q(2));
T23 = DH_computation(0, a(3), 0, q(3));
T34 = DH_computation(0, a(4), 0, q(4));


T04 = T01*T12*T23*T34;

XY4 = DirectKinematics(T04);

% Calcolo l'errore in posizione nello SO
e = XYd'-XY4;

% Definisco la matrice dei guadagni
K = kgain*eye(3);

% Calcolo le velocità desiderate nello SG (t= trasposta)
if method == "t"
    qddot = J'*K*e;

else % stiamo utilizzando l'inversa nel caso più generale perchè voglio sfruttare la ridondanza
    qa_dot = ka * W_q_progetto(q(1),q(2),q(3),q(4)); 
    
    qddot = pinv(J)*(XYddot'+K*e) + (eye(4) - pinv(J)*J)*qa_dot'; 
end
end 
