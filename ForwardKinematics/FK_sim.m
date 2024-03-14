clc,clear all,close all;
% Scara Robot FK Simulation
% Written By: Rasit Evduzen
% Date : 28-Jul-2023
%% ---------------- DH parameters ----------------

addpath('stl')
baseGeo = stlread("base.STL");
link1Geo = stlread("link1.STL");
link2Geo = stlread("link2.STL");
link3Geo = stlread("link3.STL");
linkTool = stlread("link4.STL");


% Workspace Trajectory
NoD = 90;
dd  = linspace(-30,30,NoD);
tt1 = linspace(90,-90,NoD);
tt2 = linspace(90,-90,NoD);
tt4 = linspace(90,-90,NoD);


figure('units','normalized','outerposition',[0 0 1 1],'color','w')
gif('ScaraFKSim2.gif')
for i=1:NoD
    clf
    theta1 = tt1(i);           % J1
    theta2 = tt2(i);           % J2
    d3 = dd(i);                % D3 Link Dynamic Offset
    theta4 = tt4(i);           % J4

    a = [0 90 100 0];     % link Lenght
    alpha = [0 0 0 0];    % Link Twist
    d = [117 0 d3 -45];   % Link Static Offset
    DhParam = [alpha; a; d]';             % DH Parameters
    JointVar = [theta1 theta2 0 theta4];  % Joint Parameters

%     view(40,15)% azimuth, elevation
    view((-i+180)*2,15)% azimuth, elevation
    trplot(eye(4,4),'thick',1,'rgb','length',50), hold on, grid on % Base Triad

    T04 = eye(4,4);
    AllMatrix = [];
    for i=1:size(JointVar,2)
        temp = T04;
        T04 = T04 * DHMatrixModify(DhParam(i,1),DhParam(i,2),DhParam(i,3),JointVar(i));   % (alpha,a,d,theta)
        AllMatrix = [AllMatrix; T04];
        trplot(T04,'thick',0.1,'rgb','length',50),hold on,axis equal, axis([-250 250 -250 250 0 350])
        plot3([temp(1,4) T04(1,4)],[temp(2,4) T04(2,4)],[temp(3,4) T04(3,4)],'k','LineWidth',1);
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('Scara Robot Forward Kinematic Simulation')
    end
    target_pose_fk = AllMatrix(end-3:end,:);
    % Plot Robot Geometry
    base = trisurf(baseGeo.ConnectivityList, baseGeo.Points(:, 1), baseGeo.Points(:, 2), baseGeo.Points(:, 3));
    base.FaceColor = "#EDB120";
    base.EdgeColor = "none";
    hold on
    plotLink1 = TransformGeo(link1Geo.Points, AllMatrix(1:4,:));
    plotLink2 = TransformGeo(link2Geo.Points, AllMatrix(5:8,:));
    plotLink3 = TransformGeo(link3Geo.Points, AllMatrix(9:12,:));
    plotLinkTool = TransformGeo(linkTool.Points, AllMatrix(13:end,:));
    EEPoint   = TransformGeo([0, 0, 0, 1], AllMatrix(13:end,:));

    l1 = trisurf(link1Geo.ConnectivityList, plotLink1(:, 1), plotLink1(:, 2), plotLink1(:, 3));
    l1.FaceColor = [0.5, 0.5, 0.5];
    l1.EdgeColor = "none";

    l2 = trisurf(link2Geo.ConnectivityList, plotLink2(:, 1), plotLink2(:, 2), plotLink2(:, 3));
    l2.FaceColor = "#EDB120";
    l2.EdgeColor = "none";

    l3 = trisurf(link3Geo.ConnectivityList, plotLink3(:, 1), plotLink3(:, 2), plotLink3(:, 3));
    l3.FaceColor = [0.5, 0.5, 0.5];
    l3.EdgeColor = "none";

    lTool = trisurf(linkTool.ConnectivityList, plotLinkTool(:, 1), plotLinkTool(:, 2), plotLinkTool(:, 3));
    lTool.FaceColor = "#EDB120";
    lTool.EdgeColor = "none";

    plot3(EEPoint(1, 1), EEPoint(2), EEPoint(1, 3), "b.",LineWidth=25);

    camlight;
    lightangle(250, 15);
    xlim([-300, 300]);
    ylim([-300, 300]);
    zlim([0, 265]);

    pause(eps)
    gif
end
