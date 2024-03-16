clc,clear all,close all,warning off;
% Scara Robot FK Simulation
% Written By: Rasit Evduzen
% Date : 16-Mar-2024
%% ---------------- DH parameters ----------------

addpath('stl')
baseGeo = stlread("base.STL");
link1Geo = stlread("link1.STL");
link2Geo = stlread("link2.STL");
link3Geo = stlread("link3.STL");
linkTool = stlread("link4.STL");

a = [0 90 100 0];         % link Lenght
alpha = [0 0 0 0];        % Link Twist
d = [117 0 72 -45];        % Link Static Offset
DhParam = [alpha; a; d]'; % DH Parameters
NoD = 20;
% Target Trajectory
tp1_x = linspace(100,150,NoD)';
tp1_y = linspace(-100,100,NoD)';
tp1_z = linspace(-30,-30,NoD)';
tp1_theta = linspace(0,0,NoD)';

tp2_x = linspace(150,150,NoD)';
tp2_y = linspace(100,100,NoD)';
tp2_z = linspace(-30,30,NoD)';
tp2_theta = linspace(0,0,NoD)';

tp3_x = linspace(150,100,NoD)';
tp3_y = linspace(100,-100,NoD)';
tp3_z = linspace(30,30,NoD)';
tp3_theta = linspace(0,0,NoD)';

tp4_x = linspace(100,100,NoD)';
tp4_y = linspace(-100,-100,NoD)';
tp4_z = linspace(30,-30,NoD)';
tp4_theta = linspace(0,0,NoD)';

traj = [tp1_x tp1_y tp1_z tp1_theta
        tp2_x tp2_y tp2_z tp2_theta
        tp3_x tp3_y tp3_z tp3_theta
        tp4_x tp4_y tp4_z tp4_theta];

GeometryTrans = 1;
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
for i=1:size(traj,1)
    clf
    X = traj(i,1);
    Y = traj(i,2);
    Z = traj(i,3);
    Theta = traj(i,4);

    target_pose_fk =[rotz(Theta) [X Y Z]'
    0     0     0     1];

    [J1,J2,d3,J4] = scara_ik(target_pose_fk,DhParam);
    JointVar = [J1 J2 0 J4];  % Joint Parameters
    d = [117 0 d3+72 -45];        % Link Static Offset
    DhParam = [alpha; a; d]'; % DH Parameters

    view(45,20)
    trplot(eye(4,4),'thick',1,'rgb','length',50), hold on, grid on % Base Triad
    T04 = eye(4,4);
    AllMatrix = [];
    for i=1:size(JointVar,2)
        temp = T04;
        T04 = T04 * DHMatrixModify(DhParam(i,1),DhParam(i,2),DhParam(i,3),JointVar(i));   % (alpha,a,d,theta)
        AllMatrix = [AllMatrix; T04];
        trplot(T04,'thick',0.1,'rgb','length',50),hold on,axis equal, axis([-250 250 -250 250 0 350])
        plot3([temp(1,4) T04(1,4)],[temp(2,4) T04(2,4)],[temp(3,4) T04(3,4)],'k','LineWidth',1);
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('Scara Robot Inverse Kinematic Simulation')
    end
    target_pose_ik = AllMatrix(end-3:end,:);
    plot3(traj(:,1),traj(:,2),traj(:,3)+72,'r',LineWidth=2);
    % Plot Robot Geometry
    base = trisurf(baseGeo.ConnectivityList, baseGeo.Points(:, 1), baseGeo.Points(:, 2), baseGeo.Points(:, 3),FaceAlpha=GeometryTrans);
    base.FaceColor = "#EDB120";
    base.EdgeColor = "none";
    plotLink1 = TransformGeo(link1Geo.Points, AllMatrix(1:4,:));
    plotLink2 = TransformGeo(link2Geo.Points, AllMatrix(5:8,:));
    plotLink3 = TransformGeo(link3Geo.Points, AllMatrix(9:12,:));
    plotLinkTool = TransformGeo(linkTool.Points, AllMatrix(13:end,:));
    EEPoint   = TransformGeo([0, 0, 0, 1], AllMatrix(13:end,:));

    l1 = trisurf(link1Geo.ConnectivityList, plotLink1(:, 1), plotLink1(:, 2), plotLink1(:, 3),FaceAlpha=GeometryTrans);
    l1.FaceColor = [0.5, 0.5, 0.5];
    l1.EdgeColor = "none";

    l2 = trisurf(link2Geo.ConnectivityList, plotLink2(:, 1), plotLink2(:, 2), plotLink2(:, 3),FaceAlpha=GeometryTrans);
    l2.FaceColor = "#EDB120";
    l2.EdgeColor = "none";

    l3 = trisurf(link3Geo.ConnectivityList, plotLink3(:, 1), plotLink3(:, 2), plotLink3(:, 3),FaceAlpha=GeometryTrans);
    l3.FaceColor = [0.5, 0.5, 0.5];
    l3.EdgeColor = "none";

    lTool = trisurf(linkTool.ConnectivityList, plotLinkTool(:, 1), plotLinkTool(:, 2), plotLinkTool(:, 3),FaceAlpha=GeometryTrans);
    lTool.FaceColor = "#EDB120";
    lTool.EdgeColor = "none";

    plot3(EEPoint(1, 1), EEPoint(2), EEPoint(1, 3), "b.",LineWidth=25);

    camlight;
    lightangle(250, 15);
    xlim([-300, 300]);
    ylim([-300, 300]);
    zlim([0, 265]);

    drawnow
end
