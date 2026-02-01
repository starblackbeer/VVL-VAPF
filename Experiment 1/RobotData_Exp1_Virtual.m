%% 机器人相关参数
global base;
global theta_offset;

base(:,:,1) = transl([0 0 0]) * trotz(0) %%操作机械臂
base(:,:,2) = transl([750 -1000 0]) * trotz(90); %%相机
base(:,:,3) = transl([0 0 0]); %%相机
base(:,:,4) = transl([750 1000 0]) * trotz(-90); %%操作机械臂

theta_offset = [0 -pi/2 0 0 0 0;
                0 -pi/2 0 0 0 0;
                0 -pi/2 0 0 0 0;
                0 -pi/2 0 0 0 0];
global sign;
sign = [1 1 1 1 1 1];
global robot_num;
robot_num = 1;
global obb_para;
obb_para = [100	100	0;
            50	50	0;
            75	75	0;
            55	75	0;
            25	25	85;
            50	50	0];
global a;
global d;
global alpha;
a=[0,     90, 518,   0,   0,  0];
d=[500, 0,     0,    545, 0,  109];%%此处0.111实际机械臂物理尺寸为0.109，加上0.02的原因是考虑夹持工件
alpha=[0,-pi/2,0,-pi/2,pi/2,-pi/2];
%% 机器人夹持工具相关参数
global type;
type = [2 2 2 2];
global tool_a;
global tool_d;
global tool_alpha;
global toolobb_para;
global tool_theta;
tool_a=[0,0,0,0];
tool_d=[30,50,0,0];
tool_alpha=[0,0,0,0]; 

toolobb_para = [30, tool_d(1)/2, tool_d(1)/2; 20, 20, tool_d(2)/2];
tool_theta = [0,0,0,0,0,0];
%% 碰撞检测相关参数
global space_edge;
global step; global new_space_step;
space_edge = [700 -300 500;1500 300 1000];
step = 3;new_space_step = 3;
%% 相机相关参数
global ConeGeneratrix; %圆锥面的数量
ConeGeneratrix = 30;
global R;
global depth;
R = 20;
depth = 50;
%% 控制过程变量
global update_enable; %在参数修改是，控制机械臂是否更新
update_enable = [1 1 1 1];
%% 工作平台相关参数
global workplane1;
workplane1.vertex = [500 -200 200];
workplane1.vlength = [1 0 0];
workplane1.length = 400;
workplane1.vwidth = [0 1 0];
workplane1.width = 400;
workplane1.vheight = [0 0 1];
workplane1.height = 100;
global Sphere1;
Sphere1.center = [600, -400, 300];
Sphere1.R = 150;
global workpiece1;
workpiece1.vertex = [600 -100 300];
workpiece1.vlength = [1 0 0];
workpiece1.length = 250;
workpiece1.vwidth = [0 1 0];
workpiece1.width = 250;
workpiece1.vheight = [0 0 1];
workpiece1.height = 50;
%% 创建工具箱机器人用于逆解
L1 = Link([0 d(1) a(1) alpha(1)],'modified'); 
L2 = Link([0 d(2) a(2) alpha(2)],'modified'); L2.offset = -pi/2;
L3 = Link([0 d(3) a(3) alpha(3)],'modified'); 
L4 = Link([0 d(4) a(4) alpha(4)],'modified'); 
L5 = Link([0 d(5) a(5) alpha(5)],'modified');
L6 = Link([0 d(6) a(6) alpha(6)],'modified'); 
robot_move(1)=SerialLink([L1 L2 L3 L4 L5 L6]); %连接连杆，机器人取名manman
robot_move(1).base = SE3(base(:,:,1));
robot_move(2)=SerialLink([L1 L2 L3 L4 L5 L6]); %连接连杆，机器人取名manman
robot_move(2).base = SE3(base(:,:,2));
robot_move(3)=SerialLink([L1 L2 L3 L4 L5 L6]); %连接连杆，机器人取名manman
robot_move(3).base = SE3(base(:,:,3));
robot_move(4)=SerialLink([L1 L2 L3 L4 L5 L6]); %连接连杆，机器人取名manman
robot_move(4).base = SE3(base(:,:,4));
