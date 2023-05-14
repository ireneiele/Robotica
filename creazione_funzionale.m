%% CALCOLO FUNZIONALE COSTO PROGETTO
syms q1 q2 q3 q4 'real' % dove o_x o_y o_z sono le coord dell'ostacolo,
% mentre real mi dice di considerare le variabili come val reali
q= [q1 q2 q3 q4]';
qm = [0 0 0 0]';
qM = [180 180 180 180]';

% Calcolo centro corsa
for i = 1:length(q)
    q_mean(i)=(qM(i)+qm(i))/2
end

for i = 1:length(q)
    W(i) = (((q(i)-q_mean(i))/(qM(i)-qm(i)))^2)
end

w = simplify((-1/2*length(q))*sum(W));

for i = 1:length(q)
    disp(num2str(i))
    %faccio diff, quindi derivata. In particolare faccio gradiente di una
    %funzione scalare e quindi ottengo un vettore. Quindi ottengo vettore
    %che mi punta dove la manipolabilità aumenta
    W_q(i) = simplify(diff(w, q(i)));  
end

% con queste righe di codice abbiamo calcolato la derivata di w(q) in dq
% così otteniamo la velocità qa_dot (assumendo ka=1)

matlabFunction(W_q,'File','W_q_progetto')





