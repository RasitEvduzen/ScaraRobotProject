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

% Target Trajectory
NoD = 20;
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
        T04 = T04 * DHMatrixModify(DhParam(i,1),DhParam(i,2),DhParam(i,3),JointVar(i));   % (alpha,a,d,theta)
        AllMatrix = [AllMatrix; T04];
        trplot(T04,'thick',0.1,'rgb','length',50),hold on,axis equal, axis([-250 250 -250 250 0 350])
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('Scara Robot Inverse Kinematic Simulation')
    end
    plot3(traj(:,1),traj(:,2),traj(:,3)+72,'r',LineWidth=2);
    PlotRobot(AllMatrix,baseGeo,link1Geo,link2Geo,link3Geo,linkTool)
   
    drawnow
end
