% dynamic formation selection
function index=dfs(t,ksi,Formations,h)
    gamma=0.1;
    Formations.update_estimate(ksi,gamma,h);
    index=Formations.current_index;
end