% dynamic formation selection
% decentralized case
function index=dfs(t,ksi,Formations,h,Tconv)
    gamma=0;
    Formations.update_estimate(ksi,gamma,0.001,Tconv);
    index=Formations.current_index;
end