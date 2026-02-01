%% load parameter
clc; close all;
RobotData_Exp2_Virtual;
%% Create the working range of the robot
robot1_pos_area = [500,-550,350;700,-350,450];
robot1_posture = [0,-1,0;-1,0,0;0,0,-1];
%robot2_pos_area = [400,-700,850;800,-500,860];
%robot2_pos_area = [400,-700,1050;800,-500,1060];
robot2_pos_area = [400,-700,950;800,-500,960];
%% plot
fig = figure;
xlabel('x(cm)');
ylabel('y(cm)');
zlabel('z(cm)');
hold on;
grid on;
axis equal;
view(116,1);
workpiece1_vertexes = GetCubicVertexes(workpiece1);
%% generate robot
robot = robot_generate();
%% Randomly generate the pose of Robot 1
% T1=zeros(4,4);
% T1(1,4)= robot1_pos_area(1,1) + rand() * (robot1_pos_area(2,1) - robot1_pos_area(1,1));
% T1(2,4)= robot1_pos_area(1,2) + rand() * (robot1_pos_area(2,2) - robot1_pos_area(1,2));
% T1(3,4)= robot1_pos_area(1,3) + rand() * (robot1_pos_area(2,3) - robot1_pos_area(1,3));
% T1(4,4)= 1;
% T1(1:3,1:3) = robot1_posture;
% q1 = robot_move(1).ikine(T1);
%% Randomly generate the pose of Robot 2
% T2=zeros(4,4);
% T2(1,4)= robot2_pos_area(1,1) + rand() * (robot2_pos_area(2,1) - robot2_pos_area(1,1));
% T2(2,4)= robot2_pos_area(1,2) + rand() * (robot2_pos_area(2,2) - robot2_pos_area(1,2));
% T2(3,4)= robot2_pos_area(1,3) + rand() * (robot2_pos_area(2,3) - robot2_pos_area(1,3));
% T2(4,4)= 1;
% Xvec = T2(1:3,4)' - MassCenter;
% Xvec = Xvec / norm(Xvec);
% AssitVec = [0;0;1];
% Yvec = cross(Xvec',AssitVec);
% Yvec = -Yvec / norm(Yvec);
% Zvec = cross(Xvec',Yvec);
% T2(1:3,1:3) = [Xvec',Yvec,Zvec];
% q2 = robot_move(2).ikine(T2);
%% generate VVL
MassCenter = mean(workpiece1_vertexes);
v = line_fitting_cross_point(robot(2).camera.pos,workpiece1_vertexes);
max_dist = -1; max_theta = -1;
for j = 1 : 8
    Project = Point2Line([v',robot(2).camera.pos],workpiece1_vertexes(j,:));
    cone_dist = norm(robot(2).camera.pos - Project);
    if cone_dist > max_dist 
        max_dist = cone_dist;
    end
    r = norm(workpiece1_vertexes(j,:) - Project);
    cone_theta = r / cone_dist;
    if cone_theta > max_theta 
        max_theta = cone_theta;
    end
end
dirdot = (MassCenter - robot(2).camera.pos) * v;
if dirdot < 0 
    v = -v;
end
Sphere.center = robot(2).camera.pos + v' * max_dist;
Sphere.R = max_dist * max_theta + 10;
workpiece1_fov.cone = generate_workpiece_cone(robot(2).camera, Sphere);
workpiece1_fov.cone_hp = PlotCone(workpiece1_fov.cone,ConeGeneratrix,0,[0.5 0.5 0.5],0.1);
Cylinder = GetCylinderExpress(robot(2).camera.pos, Sphere.center, 5);
hcylinder1 = PlotCylinder(Cylinder,30,0,[0.5, 0.5, 0.5],0.5);
%% Perform occlusion removal tests in sequence from the randomly generated robotic arm positions
for idx = 0 : 24
    % Update two robots
    start_theta=[AngleArray(idx*2+1,:);AngleArray(idx*2+2,:)];
    robot = robot_update(robot, start_theta);
    for j = 1 : 1 : robot_num
        robot(j) = robot_update_figure(robot(j));         
    end
    % Update VVL
    v = line_fitting_cross_point(robot(2).camera.pos,workpiece1_vertexes);
    max_dist = -1; max_theta = -1;
    for j = 1 : 8
        Project = Point2Line([v',robot(2).camera.pos],workpiece1_vertexes(j,:));
        cone_dist = norm(robot(2).camera.pos - Project);
        if cone_dist > max_dist 
            max_dist = cone_dist;
        end
        r = norm(workpiece1_vertexes(j,:) - Project);
        cone_theta = r / cone_dist;
        if cone_theta > max_theta 
            max_theta = cone_theta;
        end
    end
    dirdot = (MassCenter - robot(2).camera.pos) * v;
    if dirdot < 0 
        v = -v;
    end
    Sphere.center = robot(2).camera.pos + v' * max_dist;
    radiu_adjust = 0;
    Sphere.R = max_dist * max_theta + radiu_adjust;
    workpiece1_fov.cone = generate_workpiece_cone(robot(2).camera, Sphere);
    workpiece1_fov.cone_hp.Vertices = GetConeVertices(workpiece1_fov.cone,ConeGeneratrix);
    Cylinder = GetCylinderExpress(robot(2).camera.pos, Sphere.center, 5);
    hcylinder1.Vertices = GetCylinderVertices(Cylinder, 30);
    %% remove the occlusion
    q = robot(1).theta - robot(1).theta_offset;
    while true
        % Calculate the Cartesian Velocity based VAPF
        Vec1 = ObstaclePotentialFiled(workpiece1_fov.cone,robot(1).robotobb(4),1); %Link4
        Vec2 = ObstaclePotentialFiled(workpiece1_fov.cone,robot(1).robotobb(6),0); %Link6
        Vec3 = ObstaclePotentialFiled(workpiece1_fov.cone,robot(1).robotobb(3),0); %Link3
        [dist1,~,~,pt1] = GJK_dist(workpiece1_fov.cone_hp,robot(1).robot_shape.obb4,128);
        [dist2,~,~,pt2] = GJK_dist(workpiece1_fov.cone_hp,robot(1).robot_shape.obb6,128);
        [dist3,~,~,pt3] = GJK_dist(workpiece1_fov.cone_hp,robot(1).robot_shape.obb6,128);
        stop_distance = 0;
        if stop_distance < 0.01
            stop_distance = 0.01;
        end
        if dist1 > stop_distance && dist2 > stop_distance && dist3 > stop_distance
            break;
        elseif dist3 < stop_distance
            Vec = Vec3;
            pt = pt3;
            link_id = 3;
        elseif dist1 < stop_distance
            Vec = Vec1;
            pt = pt1;
            link_id = 4;
        elseif dist2 < stop_distance
            Vec = Vec2;
            pt = pt2;
            link_id = 6;
        end
        Vec = Vec / norm(Vec);
        VelArray(i,:) = Vec';
        magnitude=0.5;
        CartVel = [Vec(1); Vec(2); Vec(3); 0; 0; 0;] * magnitude;
        % Calculate the Joint Velocity
        TcpVel = TransformVel(CartVel, robot(1), link_id, pt);
        Jacob = robot_move(1).jacob0(q);
        qv = Jacob\TcpVel;
        q  = q + qv' * 1;
        robot = robot_update(robot, q, 1);      
        robot(1) = robot_update_figure(robot(1));
    end
end