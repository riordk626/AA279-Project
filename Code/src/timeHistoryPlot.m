function timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)

n = length(values);

fig;

for i=1:n
    subplot(n,1,i)
    p = plot(t, values{i}, 'LineWidth', 2);
    set(p, {'DisplayName'}, valueLabels{i})
    ylabel(valueNames{i})
    legend
    if i==n
        xlabel('t [sec]')
    end
    ax = gca();
    ax.FontSize = 14;
end

if exportflag
    exportgraphics(gcf, fullfile(figureName))
end