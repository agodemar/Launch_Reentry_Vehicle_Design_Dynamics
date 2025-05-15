function [ h_fig ] = plotPoint3DHelperLines(h_fig, vXYZe, options)

helpLine1X = [vXYZe(1),vXYZe(1)];
helpLine1Y = [vXYZe(2),vXYZe(2)];
helpLine1Z = [vXYZe(3),0];
helpLine2X = [vXYZe(1),vXYZe(1),0];
helpLine2Y = [0,vXYZe(2),vXYZe(2)];
helpLine2Z = [0,0,0];
hold on
plot3(helpLine1X,helpLine1Y,helpLine1Z, ...
    'Color', options.lineColor, ...
    'LineStyle', options.lineStyle, ...
    'LineWidth', options.lineWidth );
hold on
plot3(helpLine2X,helpLine2Y,helpLine2Z, ...
    'Color','k','LineStyle',':','LineWidth',1.5);

end

