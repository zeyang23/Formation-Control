function plot_log(F1)
    ts=1:F1.decision_counter - 1;
    figure()
    hold on
    for i = 1 : F1.formation_number
        plot(ts,F1.global_error_log(i,:))
    end
    title('global error of all formations')
    
    figure()
    number = 0;
    for i = 1 : F1.robot_number
        hold on
        for j = 1 : F1.formation_number
            number = number+1;
            plot(ts,F1.local_error_log{j}(i,:))
            labels{number} = ['robot index: ',num2str(i),' formation index: ',num2str(j)];
        end
    end
    legend(labels)
end