import numpy as np
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









