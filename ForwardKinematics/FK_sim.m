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

    view(40,15)% azimuth, elevation
    trplot(eye(4,4),'thick',1,'rgb','length',50), hold on, grid on % Base Triad

    T04 = eye(4,4);
    AllMatrix = [];
    for i=1:size(JointVar,2)
        temp = T04;
        T04 = T04 * DHMatrixModify(DhParam(i,1),DhParam(i,2),DhParam(i,3),JointVar(i));   % (alpha,a,d,theta)
        AllMatrix = [AllMatrix; T04];
        trplot(T04,'thick',0.1,'rgb','length',50),hold on,axis equal, axis([-250 250 -250 250 0 350])
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('Scara Robot Forward Kinematic Simulation')
    end
    target_pose_fk = AllMatrix(end-3:end,:);
    % Plot Robot Geometry
    PlotRobot(AllMatrix,baseGeo,link1Geo,link2Geo,link3Geo,linkTool)
    drawnow
end
