% Initial configuration of the robot
q0_1 = 0;
q0_2 = 0;
q0_3 = 0;

% Link lengths [m] 
L1 = 0.5;
L2 = 0.5;
L3 = 0.4;

l2 = 0.35;
l3 = 0.2;

% Control 
q1 = ris.q1;
q2 = ris.q2;
q3 = ris.q3;
F = ris.F_rob;
Fx = ris.Fx_rob;
Fy = ris.Fy_rob;
Fz = ris.Fz_rob;
cart = ris.cart_rob;

% Create the robot to display
robot = robotics.RigidBodyTree('DataFormat', 'column');

% Define the DH parameters
DH_table = [pi/2, L1, 0,  q0_1;
            0,    0,  L2, q0_2;
            0,    0,  L3, q0_3];

% Create the links and joints
link1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.HomePosition = q0_1;
joint1.JointAxis = [0 0 1];
setFixedTransform(joint1, trvec2tform([0 0 0]));
link1.Joint = joint1;
addBody(robot, link1, 'base');

link2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2', 'revolute');
joint2.HomePosition = q0_2;
joint2.JointAxis = [0 -1 0];
setFixedTransform(joint2, trvec2tform([0, 0, L1]));
link2.Joint = joint2;
addBody(robot, link2, 'link1');

link3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3', 'revolute');
joint3.HomePosition = q0_3;
joint3.JointAxis = [0 -1 0];
setFixedTransform(joint3, trvec2tform([L2, 0, 0]));
link3.Joint = joint3;
addBody(robot, link3, 'link2');

% Define the end-effector
endEffector = robotics.RigidBody('end_effector');
joint4 = robotics.Joint('fix1', 'fixed');
setFixedTransform(joint4, trvec2tform([L3, 0, 0]));
endEffector.Joint = joint4;
addBody(robot, endEffector, 'link3');

% This block of code is needed to create cylinders
for i = 1:(robot.NumBodies - 1)
    ang = pi/2;
    if i == 1
        collisionObj = collisionCylinder(0.03, L1);
        mat = axang2tform([0 0 1 ang]);
        mat(3, 4) = L1/2;
    elseif i == 2
        collisionObj = collisionCylinder(0.03, L2);
        mat = axang2tform([0 1 0 ang]);
        mat(1, 4) = L2/2;
    elseif i == 3
        collisionObj = collisionCylinder(0.03, L3);
        mat = axang2tform([0 1 0 ang]);
        mat(1, 4) = L3/2;
    end
    collisionObj.Pose = mat;
    addCollision(robot.Bodies{i}, collisionObj);
end
% Set the initial joint configuration
qs = [q1, q2, q3];
count = length(qs(:,1));
endEffectorPosition = cart;

disp(count);
%%%%%%%%%%%%%%%%%%%%% trajectory %%%%%%%%%%%%%%%%%%%%%
% Parameters
r = 0.3;
w = pi/9;
x_0 = -0.3;
y_0 = 0.5;
z_0 = 0.5;
T1 = 10;
T2 = 20;
T3 = 30;

% Time points
t = linspace(0, T3, 1000); % Adjust the number of points for smoother plotting

% Initialize arrays to store the positions
x = zeros(size(t));
y = zeros(size(t));
z = zeros(size(t));

% Calculate positions for each time point
for i = 1:length(t)
    if t(i) >= 0 && t(i) <= T1
        theta = w*t(i);
        x(i) = x_0 + r * cos(theta);
        y(i) = y_0; 
        z(i) = z_0 + r * sin(theta);
    elseif t(i) >= T1 && t(i) <= T2
        theta = w*T1;
        x(i) = x_0 + r*cos(theta);
        y(i) = y_0;
        z(i) = z_0 + r*sin(theta);
    elseif t(i) >= T2 && t(i) <= T3
        theta = w*T1;
        x(i) = x_0 + r*cos(theta) - 0.1 * (t(i) - T2) / (T3 - T2);
        y(i) = y_0 - 0.4* (t(i) - T2) / (T3 - T2);
        z(i) = z_0 + r*sin(theta);
    end
end

% Define segments based on time intervals
segment1 = find(t >= 0 & t <= T1);
segment2 = find(t >= T1 & t <= T2);
segment3 = find(t >= T2 & t <= T3);

% Define colors for each segment
color1 = [0, 1, 0]; % Orange (red is made slightly orange)
color2 = [1, 0, 0];   % Green
color3 = [0.3, 0.3, 0.8]; % Darker blue (lower blue component)

% Create an array to specify colors for each segment
colors = zeros(length(t), 3); % Initialize as zeros
colors(segment1, :) = repmat(color1, length(segment1), 1); 
colors(segment3, :) = repmat(color3, length(segment3), 1); 

marker_size = 5;

%%%%%%%%%%%%%%%%% forces %%%%%%%%%%%%%%%

% Plot the trajectory with different colors and marker sizes
figure;
show(robot,qs(1,:)','Collisions','on','Visuals','off');

view(3)
ax = gca;
ax.Projection = 'orthographic';
hold on
scatter3(x(segment1), y(segment1), z(segment1), marker_size, colors(segment1, :), 'filled');
hold on;
scatter3(x(segment2), y(segment2), z(segment2), 100, color2, 'filled');
scatter3(x(segment3), y(segment3), z(segment3), marker_size, colors(segment3, :), 'filled');


xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trajectory');

grid on;

% Display the robot in the initial configuration
framesPerSecond = count/simulation_time;
%framesPerSecond = simulation_time/(count-1);
disp(framesPerSecond);

r = rateControl(framesPerSecond);
for i = 1:count
    % Update the robot visualization with the current configuration
    show(robot, qs(i, :)', 'Collisions', 'on', 'Visuals', 'off', 'PreservePlot', false);
    
    current_Force = F(:,:,i);
    q = qs(i,:);
    current_endEffectorPosition = endEffectorPosition(:,:,i);
    

    if current_Force(1) == 0 && current_Force(2) == 0 && current_Force(3) == 0
        current_endEffectorPosition = endEffectorPosition(:,:,i);
        
    end
    if current_Force(1) == -30 && current_Force(2) == -20 && current_Force(3) == 0
            current_endEffectorPosition = endEffectorPosition(:,:,i);
    end

    if current_Force(1) == 0 && current_Force(2) == 0 && current_Force(3) == -35              
               current_endEffectorPosition = [cos(q(1))*l2*cos(q(2))...
                ; l2*sin(q(1))*cos(q(2)); ...
                L1 + l2*sin(q(2))];

    end


    h = quiver3(current_endEffectorPosition(1), current_endEffectorPosition(2), current_endEffectorPosition(3), ...
        current_Force(1), current_Force(2), current_Force(3), 0.03);
    set(h, 'Color',[1,0,0]);
    drawnow;
    waitfor(r);
    delete(h);
end

hold off;