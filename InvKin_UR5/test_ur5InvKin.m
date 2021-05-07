%% Inverse Kinematics Test
rosshutdown;
ur5 = ur5_interface();
joint_offset = [-pi/2 -pi/2 0 -pi/2 0 0]';
joints = [pi/7 pi/4 pi/4 pi/6 pi/4 0]';
g_S_T = ur5FwdKin(joints);
%g_baseK_S = [ROTZ(pi/2) [0 0 0.0892]'; 0 0 0 1];  %transformation from keating base to {S}
g_baseK_S = [eye(3) [0 0 0]'; 0 0 0 1];  %transformation from keating base to {S}
g_baselink_s = [eye(3) [0 0 0]';0 0 0 1];

%% define S frame
sFrame = tf_frame('base_link', 'S', g_baselink_s);
%% define T frame
tFrame = tf_frame('ee_link', 'T', eye(4));
%% -90 degree rotation around z and up x 0.0892 
baseKFrame = tf_frame('S','base_K',eye(4));
baseKFrame.move_frame('S',inv(g_baseK_S));

g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1]; %transformation from {T} to keating tool 
%% -90 around x and 90 around y
toolKFrame = tf_frame('T','tool_K',eye(4));
toolKFrame.move_frame('T',(g_T_toolK));

g_des = g_baseK_S*g_S_T*g_T_toolK; %transformation from keating base to keating tool 
thetas = ur5InvKin(g_des);

% from Denavit-Hartenberg convention to lab3 convention
final_joints = thetas - joint_offset
%% test 
joint_angle = joints
gs2 = ur5FwdKin(final_joints(:,1));
error = abs(gs2 - g_S_T)