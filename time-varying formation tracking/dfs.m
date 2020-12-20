% dynamic formation selection
% decentralized case
function index=dfs(t,ksi,Formations,h,Tconv)
    gamma=0.1;
    Formations.update_estimate(ksi,gamma,h,Tconv);
    index=Formations.current_index;
end