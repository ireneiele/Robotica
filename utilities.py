import numpy as np



def DH_computation(d, a, alpha, theta):
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                 [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                 [0, np.sin(alpha), np.cos(alpha), d],
                 [0, 0, 0, 1]])
    return T

def DirectKinematics(T):

    # Estraggo dalla trasformazione omogenea il vettore traslazione
    x = T[0:3,3]

    # Estraggo dalla trasformazione omogenea il quaternione unitario
    sgn_1 = np.sign(T[2,1]-T[1,2])
    sgn_2 = np.sign(T[0,2]-T[2,0])
    sgn_3 = np.sign(T[1,0]-T[0,1])

    # Calcolo i valori per i quali e definito il quaternione Q={eta,epsilon}
    eta = np.sqrt(T[0,0]+T[1,1]+T[2,2]+1)/2
    eps = 0.5*np.array([sgn_1*np.sqrt(T[0,0]-T[1,1]-T[2,2]+1),
                        sgn_2*np.sqrt(-T[0,0]+T[1,1]-T[2,2]+1),
                        sgn_3*np.sqrt(-T[0,0]-T[1,1]+T[2,2]+1)])

    Q = np.hstack([eta, eps]) # Mettiamo i valori insieme tramite concatenazione verticale

    return x, Q      #restituisce posa e orientamento 



# Funzione per calcolare segmento rettilineo e poi circonferenza nello spazio Cartesiano
def cartesianPlanner(pA,pB,pC,ro,deg,Cen,tA,tB,tC,t):        
    if t <= tA:
        pd = pA
        pdot = np.zeros(3)
    
    elif tA<t <= tB:
        # Questa parte serve a rappresentare la traslazione da A a B
        A = np.array([[tA**3, tA**2, tA, 1],
            [tB**3, tB**2, tB, 1],
            [3*tA**2, 2*tA, 1, 0],
            [3*tB**2, 2*tB, 1, 0]])

        b = np.array([[np.linalg.norm(pA)],[np.linalg.norm(pB-pA)],[0],[0]])

        x = np.linalg.solve(A, b)

        s = x[0]*t**3+x[1]*t**2+x[2]*t+x[3]
        sdot = 3*x[0]*t**2+2*x[1]*t+x[2]
        
        pd = pA + s*(pB-pA)/np.linalg.norm(pB-pA)
        pdot = sdot*(pB-pA)/np.linalg.norm(pB-pA)
        
    elif tB<t <= tC:
        # La funzione che traccia l'arco di circonferenza sta sotto va solo implementato coi nomi corretti
        R = np.array([0, 1, 0],[1,0,0],[0,0,-1])
        
        A = np.array([[tB**3, tB**2, tB, 1],
                    [tC**3, tC**2, tC, 1],
                    [3*tB**2, 2*tB, 1, 0],
                    [3*tC**2, 2*tC, 1, 0]])
        
        b = np.array([np.linalg.norm(pB)],ro*deg,[np.linalg.norm(pdot)],0)
        
        x = np.linalg.solve(A, b)
        
        s = x[0]*t**3+x[1]*t**2+x[2]*t+x[3]
        sdot = 3*x[0]*t**2+2*x[1]*t+x[2]
        
        pd = Cen+ R*np.array([ro*np.cos(s/ro)],[ro*np.sin(s/ro)],0)
        pdot = R*np.array([-sdot*np.sin(s/ro)],[sdot*np.cos(s/ro)],0)
 
    elif t > tC:
        pd = pC
        pdot = np.zeros(3)     
    return pd, pdot

def Jacobian_4dof(q, a):
    T01 = DH_computation(0, a(1), 0, q(1))
    T12 = DH_computation(0, a(2), 0, q(2))
    T23 = DH_computation(0, a(3), 0, q(3))
    T34 = DH_computation(0, a(4), 0, q(4))
    
    T02 = np.matmul(T01,T12)#fa il prodotto tra 2 arrays
    T03 = np.matmul(T02,T23)   
    T04 = np.matmul(T03,T34)
    
    z0 = np.array([0,0,1])
    z1 = T01[0:3,2]
    z2 = T02[0:3,2]
    z3 = T03[0:3,2]
    
    p0 = np.array([0,0,0])     
    p1 = T01[0:3,3]
    p2 = T02[0:3,3]
    p3 = T03[0:3,3]
    p = T04[0:3,3]
    
      # Calcolo dei singoli jacobiani per arrivare alla definizione dello jacobiano finale
    J1 = np.cross(z0,p-p0)
    J2 = np.cross(z1,p-p1)
    J3 = np.cross(z2,p-p2)
    J4 = np.cross(z3,p-p3)

    Jac = np.vstack((J1, J2, J3, J4)) # Concatenazione Jacobiani

    return Jac.T

def W_q_progetto(q1,q2,q3,q4):
    W_q = np.array([q1*(-1.234567901234568e-4)+1.0/9.0e+1,q2*(-1.234567901234568e-4)+1.0/9.0e+1,q3*(-1.234567901234568e-4)+1.0/9.0e+1,q4*(-1.234567901234568e-4)+1.0/9.0e+1])


def InverseKinematicsProgetto(q, a, XYd, XYddot, kgain, method, ka):
    # Calcolo lo Jacobiano geometrico
    J = Jacobian_4dof(q, a)

    # Calcolo la posizione attuale
    T01 = DH_computation(0, a(1), 0, q(1))
    T12 = DH_computation(0, a(2), 0, q(2))
    T23 = DH_computation(0, a(3), 0, q(3))
    T34 = DH_computation(0, a(4), 0, q(4))
    
    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)
    XY4 = DirectKinematics(T04)

    # Calcolo l'errore in posizione nello SO
    e = XYd-XY4

    # Definisco la matrice dei guadagni
    K = np.matmul(kgain,np.eye(3))

    # Calcolo le velocità desiderate nello SG (t= trasposta)
    if method == "t":
        qddot = np.matmul(np.matmul(J,K),e)

    else: #stiamo utilizzando l'inversa nel caso più generale perchè voglio sfruttare la ridondanza
        qa_dot = np.matmul(ka,W_q_progetto(q(1),q(2),q(3),q(4)))
        
        Ke = np.matmul(K,e)
        P = np.eye(4) - np.matmul(np.linalg.pinv(J),J)
        
        qddot = np.matmul(np.linalg.pinv(J),XYddot+Ke) + np.matmul(P,qa_dot)
    
