%Define the RigidBodyTree
rbt = rigidBodyTree('DataFormat','column','MaxNumBodies',6);

%Define the rigid bodies you want to use
body1 = rigidBody('body1');
body2 = rigidBody('body2');
body3 = rigidBody('body3');
body4 = rigidBody('body4');
body5 = rigidBody('body5');
body6 = rigidBody('body6');


%Define the joints and the joint type for each body
jnt1 = rigidBodyJoint('jnt1','revolute');

%Create a homogeneous transformation given a vector (X,Y,Z)

%Translation Vector
V = [[0.1]
     [0.0]
     [0.0]
     [0.0]]
 
%Homogenous Transformation Matrix   
T = [[1.0, 0.0, 0.0, 0.0]
     [0.0, 1.0, 0.0, 0.0]
     [0.0, 0.0, 1.0, 0.0]
     [0.0, 0.0, 0.0, 1.0]]

%Insert the Translation Vector(V) into the Homogenous Matrix(T)
T(:,4) = T(:,4) + V

setFixedTransform(jnt1,T)


%Do the same thing for each joint
jnt2 = rigidBodyJoint('jnt2','revolute');

V2 = [[0.15]
     [0.015]
     [0.0]
     [0.0]]

T2 = [[1.0, 0.0, 0.0, 0.0]
     [0.0, 1.0, 0.0, 0.0]
     [0.0, 0.0, 1.0, 0.0]
     [0.0, 0.0, 0.0, 1.0]]

T2(:,4) = T2(:,4) + V2

setFixedTransform(jnt2,T2)


%Joint 3
jnt3 = rigidBodyJoint('jnt3','revolute');

V3 = [[0.1]
     [0.04]
     [0.0]
     [0.0]]

T3 = [[1.0, 0.0, 0.0, 0.0]
     [0.0, 1.0, 0.0, 0.0]
     [0.0, 0.0, 1.0, 0.0]
     [0.0, 0.0, 0.0, 1.0]]

T3(:,4) = T3(:,4) + V3

setFixedTransform(jnt3,T3)


%Joint 4
jnt4 = rigidBodyJoint('jnt4','revolute');

V4 = [[0.1]
     [0.015]
     [0.0]
     [0.0]]

T4 = [[1.0, 0.0, 0.0, 0.0]
     [0.0, 1.0, 0.0, 0.0]
     [0.0, 0.0, 1.0, 0.0]
     [0.0, 0.0, 0.0, 1.0]]

T4(:,4) = T4(:,4) + V4

setFixedTransform(jnt4,T4)


%Joint 5
jnt5 = rigidBodyJoint('jnt5','revolute');

V5 = [[0.015]
     [0.1]
     [0.0]
     [0.0]]

T5 = [[1.0, 0.0, 0.0, 0.0]
     [0.0, 1.0, 0.0, 0.0]
     [0.0, 0.0, 1.0, 0.0]
     [0.0, 0.0, 0.0, 1.0]]

T5(:,4) = T5(:,4) + V5

setFixedTransform(jnt5,T5)


%Joint 6
jnt6 = rigidBodyJoint('jnt6', 'revolute');

V6 = [[0.1]
     [0.15]
     [0.0]
     [0.0]]

T6 = [[1.0, 0.0, 0.0, 0.0]
     [0.0, 1.0, 0.0, 0.0]
     [0.0, 0.0, 1.0, 0.0]
     [0.0, 0.0, 0.0, 1.0]]

T6(:,4) = T6(:,4) + V6

setFixedTransform(jnt6,T6)


%Define an end effector
endeffector = rigidBody('endeffector');

endeffector.addFrame('toolTip','endeffector', trvec2tform([0.1, 0.1, 0.0]));

endeffectorjnt = rigidBodyJoint('endeffectorjnt','fixed');

%Access the rigid bodies joint properties and 
%assign each one its own joint property
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
endeffector.Joint = endeffectorjnt;

%Add each body in sequence
addBody(rbt,body1,'base');
addBody(rbt,body2,'body1');
addBody(rbt,body3,'body2');
addBody(rbt,body4,'body3');
addBody(rbt,body5,'body4');
addBody(rbt,body6,'body5');
addBody(rbt,endeffector,'body6');


showdetails(rbt)
show(rbt)


%Create a circle that can be traced
t = (0:0.2:10)'; %Time
count = length(t);
center = [0.565 0.32 0];
radius = 0.13;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

q0 = homeConfiguration(rbt);
qInitial = q0; % Use home configuration as the initial guess
ndof = length(q0);
qs = zeros(count, ndof);


ik = inverseKinematics('RigidBodyTree', rbt);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'toolTip'


for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end


figure
show(rbt,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2))
axis([-0.1 1 -0.3 1 -0.2 1])


framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(rbt,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
