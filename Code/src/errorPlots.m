function errorPlots(fig, t, meanValues, errorValues, valueNames, figureName, exportflag)

n = length(meanValues);

fig;

for i=1:n
    subplot(n,1,i)
    p = plot(t, meanValues{i}, 'b', 'LineWidth', 2);
    % set(p, {'DisplayName'}, valueLabels{i})
    hold on
    % plot(t, errorValues{i}, color, 'LineWidth', 2, 'LineStyle', '--')
    % plot(t, -errorValues{i}, color, 'LineWidth', 2, 'LineStyle', '--')
    patch([t(:); flipud(t(:))]', [meanValues{i}(:)+errorValues{i}(:);...
        flipud(meanValues{i}(:)-errorValues{i}(:))]', 'k', 'FaceAlpha',0.2) 
    ylabel(valueNames{i})
    if i==n
        xlabel('t [sec]')
    end
    ax = gca();
    ax.FontSize = 14;
end

if exportflag
    exportgraphics(gcf, fullfile(figureName))
end