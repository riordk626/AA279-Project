function errorPlots(fig, t, meanValues, trueValues, errorValues, valueNames, figureName, exportflag)

n = length(meanValues);

fig;

for i=1:n
    subplot(n,1,i)
    plot(t, meanValues{i}, 'b', 'LineWidth', 2, 'DisplayName', '$\mu$');
    % set(p, {'DisplayName'}, valueLabels{i})
    hold on
    plot(t, trueValues{i}, 'r', 'LineWidth', 2, 'DisplayName', '$x$')
    patch([t(:); flipud(t(:))]', [meanValues{i}(:)+errorValues{i}(:);...
        flipud(meanValues{i}(:)-errorValues{i}(:))]', 'k', 'FaceAlpha',0.2, 'HandleVisibility', 'off') 
    ylabel(valueNames{i}, 'interpreter', 'latex')
    legend('interpreter', 'latex')
    if i==n
        xlabel('t [sec]')
    end
    ax = gca();
    ax.FontSize = 14;
end

if exportflag
    exportgraphics(gcf, fullfile(figureName))
end