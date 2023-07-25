n = 20;
for i=1:n
    XYZ = randn(100,3);
    scatter3(XYZ(:,1), XYZ(:,2), XYZ(:,3), 'filled');
    set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3]);
    pause(0.1);
end
