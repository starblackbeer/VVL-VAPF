%% Load the relevant parameters of the robot
clear all; clc; close all;
RobotData_Exp1_Virtual;
%% plot
fig = figure;
xlabel('x');
ylabel('y');
zlabel('z');
hold on;
grid on;
axis equal;
view(118,12);
%% Generate the workpiece activity space and robot
workpiece1_hp = PlotCubic(workpiece1,[0.6, 0.1, 0.1],1);  %打印工件
workpiece1_vertexes = GetCubicVertexes(workpiece1);
robot = robot_generate();
robot1_posture = [0,-1,0;-1,0,0;0,0,-1];
%% Generate the camera model
T1 = transl(1200,0,950) * trotx(-90) * troty(-90) * trotx(-45);
cam1 = CentralCamera('focal', 0.008, 'pixel', 10e-6, ...
    'resolution', [3072 2048], 'centre', [1536 1024], 'name', 'mycamera','pose',T1);
pos = [1200;0;930];
euler = [0 -pi/3.5 0];
camera = CameraGenerate(euler, pos);
%% Generate the VVL
v = line_fitting_cross_point(cam1.T.t,workpiece1_vertexes);
max_dist = -1; max_theta = -1; max_r = -1;
for j = 1 : 8
    Project = Point2Line([v',cam1.T.t'],workpiece1_vertexes(j,:));
    cone_dist = norm(cam1.T.t' - Project);
    if cone_dist > max_dist 
        max_dist = cone_dist;
    end
    r = norm(workpiece1_vertexes(j,:) - Project);
    cone_theta = r / cone_dist;
    if cone_theta > max_theta 
        max_theta = cone_theta;
    end
    if r > max_r 
        max_r = r;
    end
end
MassCenter = mean(workpiece1_vertexes);
dirdot = (MassCenter - cam1.T.t') * v;
if dirdot < 0 
    v = -v;
end
Sphere.center = cam1.T.t' + v' * max_dist;
Sphere.R = max_dist * max_theta;
Sampling.center = cam1.T.t' + v' * max_dist;
Sampling.R = max_r;
Cone = GetConeExpress(Sampling.center, cam1.T.t', Sampling.R);
VCone = GetConeExpress(Sphere.center, cam1.T.t', Sphere.R);
hcone = PlotCone(VCone,30,0,[0.5, 0.5, 0.5],0.3);
%% Uniform sampling is used to obtain position points
index = 1;
Point = zeros(10000,3);
for h = Cone.h * 3 / 4 : -75 : Cone.h * 2 / 5
    radiu = Cone.r;
    for r = radiu * 9 / 7 : -30 : radiu * 4 / 7
        for theta = pi * 7 / 8 : pi / 8 : pi * 17 / 8
            Point(index,:) = Rodriguez(Cone.vertex + Cone.hvec * h + r * Cone.rvec,Cone.vertex + Cone.hvec * h,Cone.hvec,theta);
            index = index + 1;
        end
    end
end
plot3(Point(:,1),Point(:,2),Point(:,3), '.');
%% Move the robot to these positions and determine the occlusion relationship
for select_index = 1 : index - 1
    T1=zeros(4,4);
    T1(1:3,4)= Point(select_index,:)';
    T1(4,4)= 1;
    T1(1:3,1:3) = robot1_posture;
    q1 = robot_move(1).ikine(T1);
    start_theta=q1;
    robot = robot_update(robot, start_theta);
    for j = 1 : 1 : robot_num
        robot(j) = robot_update_figure(robot(j));          %更新机器人图形参数
    end
    %% 判断碰撞关系
    flag(select_index) = 0;
    [dist1,~,~,~] = GJK_dist(robot(1).robot_shape.obb1,hcone,64);
    [dist2,~,~,~] = GJK_dist(robot(1).robot_shape.obb2,hcone,64);
    [dist3,~,~,~] = GJK_dist(robot(1).robot_shape.obb3,hcone,64);
    [dist4,~,~,~] = GJK_dist(robot(1).robot_shape.obb4,hcone,64);
    [dist5,~,~,~] = GJK_dist(robot(1).robot_shape.obb5,hcone,64);
    [dist6,~,~,~] = GJK_dist(robot(1).robot_shape.obb6,hcone,64);
    if dist <= 0.01 || dist2 <= 0.01 || dist3 <= 0.01 || dist4 <= 0.01 || dist5 <= 0.01 || dist6 <= 0.01
        flag(select_index) = 1;
    end
end
%% 相机拍照，判断机械臂和工作空间的实际遮挡关系
% % 工作空间
% s1 = [workpiece1.length workpiece1.width workpiece1.height];
% T1 = zeros(4,4);
% T1(1:3,1:3) = [workpiece1.vlength',workpiece1.vwidth',workpiece1.vheight'];
% center1 = workpiece1.vertex + workpiece1.length * workpiece1.vlength / 2 + workpiece1.width * workpiece1.vwidth / 2 + workpiece1.vheight * workpiece1.height / 2; 
% T1(1:3,4) = center1';
% T1(4,4) = 1;
% [X1,Y1,Z1] = mkcube(s1, 'pose', T1, 'edge');
% 
% % 机械臂连杆4
% s2 = [robot(1).robotobb(4).length robot(1).robotobb(4).width robot(1).robotobb(4).height];
% T2 = zeros(4,4);
% T2(1:3,1:3) = [robot(1).robotobb(4).vlength',robot(1).robotobb(4).vwidth',robot(1).robotobb(4).vheight'];
% center2 = robot(1).robotobb(4).vertex + robot(1).robotobb(4).length * robot(1).robotobb(4).vlength / 2 ...
%                                       + robot(1).robotobb(4).width  * robot(1).robotobb(4).vwidth / 2  ...
%                                       + robot(1).robotobb(4).vheight * robot(1).robotobb(4).height / 2; 
% T2(1:3,4) = center2';
% T2(4,4) = 1;
% [X2,Y2,Z2] = mkcube(s2, 'pose', T2, 'edge');
% 
% % 机械臂连杆6
% s3 = [robot(1).robotobb(6).length robot(1).robotobb(6).width robot(1).robotobb(6).height];
% T3 = zeros(4,4);
% T3(1:3,1:3) = [robot(1).robotobb(6).vlength',robot(1).robotobb(6).vwidth',robot(1).robotobb(6).vheight'];
% center3 = robot(1).robotobb(6).vertex + robot(1).robotobb(6).length * robot(1).robotobb(6).vlength / 2 ...
%                                       + robot(1).robotobb(6).width  * robot(1).robotobb(6).vwidth / 2  ...
%                                       + robot(1).robotobb(6).vheight * robot(1).robotobb(6).height / 2; 
% T3(1:3,4) = center3';
% T3(4,4) = 1;
% [X3,Y3,Z3] = mkcube(s3, 'pose', T3, 'edge');
% 
% % 机械臂连杆3
% s4 = [robot(1).robotobb(3).length robot(1).robotobb(3).width robot(1).robotobb(3).height];
% T4 = zeros(4,4);
% T4(1:3,1:3) = [robot(1).robotobb(3).vlength',robot(1).robotobb(3).vwidth',robot(1).robotobb(3).vheight'];
% center4 = robot(1).robotobb(3).vertex + robot(1).robotobb(3).length * robot(1).robotobb(3).vlength / 2 ...
%                                       + robot(1).robotobb(3).width  * robot(1).robotobb(3).vwidth / 2  ...
%                                       + robot(1).robotobb(3).vheight * robot(1).robotobb(3).height / 2; 
% T4(1:3,4) = center4';
% T4(4,4) = 1;
% [X4,Y4,Z4] = mkcube(s4, 'pose', T4, 'edge');
% 
% % 机械臂连杆1
% s5 = [robot(1).robotobb(1).length robot(1).robotobb(1).width robot(1).robotobb(1).height];
% T5 = zeros(4,4);
% T5(1:3,1:3) = [robot(1).robotobb(1).vlength',robot(1).robotobb(1).vwidth',robot(1).robotobb(1).vheight'];
% center5 = robot(1).robotobb(1).vertex + robot(1).robotobb(1).length * robot(1).robotobb(1).vlength / 2 ...
%                                       + robot(1).robotobb(1).width  * robot(1).robotobb(1).vwidth / 2  ...
%                                       + robot(1).robotobb(1).vheight * robot(1).robotobb(1).height / 2; 
% % 机械臂连杆2
% T5(1:3,4) = center5';
% T5(4,4) = 1;
% [X5,Y5,Z5] = mkcube(s5, 'pose', T5, 'edge');
% % 机械臂连杆2
% s6 = [robot(1).robotobb(1).length robot(1).robotobb(1).width robot(1).robotobb(2).height];
% T6 = zeros(4,4);
% T6(1:3,1:3) = [robot(1).robotobb(2).vlength',robot(1).robotobb(2).vwidth',robot(1).robotobb(2).vheight'];
% center6 = robot(1).robotobb(1).vertex + robot(1).robotobb(2).length * robot(1).robotobb(2).vlength / 2 ...
%                                       + robot(1).robotobb(2).width  * robot(1).robotobb(2).vwidth / 2  ...
%                                       + robot(1).robotobb(2).vheight * robot(1).robotobb(2).height / 2; 
% T6(1:3,4) = center6';
% T6(4,4) = 1;
% [X6,Y6,Z6] = mkcube(s6, 'pose', T6, 'edge');
% % 机械臂连杆5
% s7 = [robot(1).robotobb(1).length robot(1).robotobb(1).width robot(1).robotobb(2).height];
% T7 = zeros(4,4);
% T7(1:3,1:3) = [robot(1).robotobb(5).vlength',robot(1).robotobb(5).vwidth',robot(1).robotobb(5).vheight'];
% center7 = robot(1).robotobb(1).vertex + robot(1).robotobb(5).length * robot(1).robotobb(5).vlength / 2 ...
%                                       + robot(1).robotobb(5).width  * robot(1).robotobb(5).vwidth / 2  ...
%                                       + robot(1).robotobb(5).vheight * robot(1).robotobb(5).height / 2; 
% T7(1:3,4) = center7';
% T7(4,4) = 1;
% [X7,Y7,Z7] = mkcube(s7, 'pose', T7, 'edge');
% %% 投影相机模型
% cam1.mesh(X1, Y1, Z1);
% cam1.mesh(X2, Y2, Z2);
% cam1.mesh(X3, Y3, Z3);
% cam1.mesh(X4, Y4, Z4);
% cam1.mesh(X5, Y5, Z5);
% cam1.mesh(X6, Y6, Z6);
% cam1.mesh(X7, Y7, Z7);