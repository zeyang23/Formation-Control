function ksi_dot = controller_3d(t,ksi,K1_single,cal_h,cal_hvdot,L)
    % ksi (6*N,1) vector
    % K1  (2,1)   vector
    % cal_h: function. return (6*N,1) vector
    % cal_hvdot: function. return (3*N,1) vector
    % L 3d array all the interaction Laplacians
    
    N=size(ksi,1)/6;
    
    B2_single=[0;1];
    B1_single=[1;0];
    
    B2 = kron(eye(3),B2_single);
    B1 = kron(eye(3),B1_single);
    
    P_single = solveP(K1_single);

    K1 = kron(eye(3),K1_single);
    P=kron(eye(3),P_single);

    lamda = lamda_min(L);
    K2_single = (2*lamda)^(-1)*B2_single'*P_single;

    K2 = kron(eye(3),K2_single);
    
    H = cal_h(t,ksi);

    Hv_dot = cal_hvdot(t,ksi);

    % choose a Laplacian in L
    index=1;
    
    ksi_dot = (kron(eye(N),(B2*K1+B1*B2'))-kron(L(:,:,index),B2*K2))*ksi...
    -(kron(eye(N),B2*K1)-kron(L(:,:,index),B2*K2))*H+kron(eye(N),B2)*Hv_dot;
end