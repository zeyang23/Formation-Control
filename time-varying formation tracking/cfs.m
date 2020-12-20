% dynamic formation selection.
% centralized case
function index=cfs(t,ksi,Formations)
    Formations.centralized_decision(ksi);
    index=Formations.current_index;
end