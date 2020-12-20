% controller for dynamic formation selection.
% centralized case
function ksi_dot = controller_cfs(t,ksi,K1_single,Formations)

    % K1  (2,1)   vector
    % cal_h: function. return (4*N,1) vector
    % cal_hvdot: function. return (2*N,1) vector
    % L 3d array all the interaction Laplacians
    
    
    %% dynamic formation selection
    index=cfs(t,ksi,Formations);
    L=Formations.Laplacians;
    
    Targets=Formations.Targets;
    
    H=get_ksi(Targets(:,:,index));
    
    Hv_dot=zeros(2*Formations.robot_number,1);
    
    
    %% formation tracking controller
    N=size(ksi,1)/4;
    
    B2_single=[0;1];
    B1_single=[1;0];
    
    B2 = kron(eye(2),B2_single);
    B1 = kron(eye(2),B1_single);
    
    P_single = solveP(K1_single);

    K1 = kron(eye(2),K1_single);
    P=kron(eye(2),P_single);

    lamda = lamda_min(L);
    K2_single = (2*lamda)^(-1)*B2_single'*P_single;

    K2 = kron(eye(2),K2_single);
    
    
    ksi_dot = (kron(eye(N),(B2*K1+B1*B2'))-kron(L(:,:,index),B2*K2))*ksi...
    -(kron(eye(N),B2*K1)-kron(L(:,:,index),B2*K2))*H+kron(eye(N),B2)*Hv_dot;
    
    %% obstacle avoidance modification
    ksi_dot=ksi_dot+obstacle(ksi);
    
end