function [] = PlotRobot(TransPlot,baseGeo,link1Geo,link2Geo,link3Geo,linkTool)

% Plot Robot Geometry
base = trisurf(baseGeo.ConnectivityList, baseGeo.Points(:, 1), baseGeo.Points(:, 2), baseGeo.Points(:, 3));
base.FaceColor = "#EDB120";
base.EdgeColor = "none";
plotLink1 = TransformGeo(link1Geo.Points, TransPlot(1:4,:));
plotLink2 = TransformGeo(link2Geo.Points, TransPlot(5:8,:));
plotLink3 = TransformGeo(link3Geo.Points, TransPlot(9:12,:));
plotLinkTool = TransformGeo(linkTool.Points, TransPlot(13:end,:));
EEPoint   = TransformGeo([0, 0, 0, 1], TransPlot(13:end,:));

trisurf(link1Geo.ConnectivityList, plotLink1(:, 1), plotLink1(:, 2), plotLink1(:, 3), ...
    FaceColor = [0.5, 0.5, 0.5],EdgeColor = "none");

trisurf(link2Geo.ConnectivityList, plotLink2(:, 1), plotLink2(:, 2), plotLink2(:, 3),...
    FaceColor = "#EDB120",EdgeColor = "none");

trisurf(link3Geo.ConnectivityList, plotLink3(:, 1), plotLink3(:, 2), plotLink3(:, 3),...
    FaceColor = [0.5, 0.5, 0.5],EdgeColor = "none");

trisurf(linkTool.ConnectivityList, plotLinkTool(:, 1), plotLinkTool(:, 2), plotLinkTool(:, 3),...
    FaceColor = "#EDB120",EdgeColor = "none");
plot3(EEPoint(1, 1), EEPoint(2), EEPoint(1, 3), "b.",LineWidth=25);

camlight;
lightangle(250, 15);
xlim([-300, 300]);
ylim([-300, 300]);
zlim([0, 300]);

end