function timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, legendflag, exportflag)

n = length(values);

fig;

for i=1:n
    subplot(n,1,i)
    p = plot(t, values{i}, 'LineWidth', 2);
    if legendflag
        set(p, {'DisplayName'}, valueLabels{i})
        legend('Interpreter','latex')
    end
    ylabel(valueNames{i}, 'Interpreter', 'latex')
    if i==n
        xlabel('t [sec]')
    end
    ax = gca();
    ax.FontSize = 14;
end

if exportflag
    exportgraphics(gcf, figureName)
    saveas(gcf, figureName)
end