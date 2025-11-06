%Define the RigidBodyTree
rbt = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

%Define the rigid bodies you want to use
body1 = rigidBody('body1');
body2 = rigidBody('body2');
body3 = rigidBody('body3');
body4 = rigidBody('body4');

%Define the joints and the joint type for each body
jnt1 = rigidBodyJoint('jnt1','fixed');

%Create a homogeneous transformation given a vector (X,Y,Z)
transform = trvec2tform([0.1,0.1,0.5])
setFixedTransform(jnt1,transform)

%Do the same thing for each joint
jnt2 = rigidBodyJoint('jnt2','revolute');
transform2 = trvec2tform([0.1,0.1,0.1])
setFixedTransform(jnt2,transform2)
jnt2.HomePosition = pi/64

%Joint 3
jnt3 = rigidBodyJoint('jnt3','revolute');
transform3 = trvec2tform([0.1,0.1,0])
setFixedTransform(jnt3,transform3)

%Joint 4
jnt4 = rigidBodyJoint('jnt4','revolute');
setFixedTransform(jnt4,transform3)
jnt4.HomePosition = pi/64

%Define an end effector
endeffector = rigidBody('endeffector');
endeffector.addFrame('toolTip','endeffector',trvec2tform([0.1, 0.1, 0]));

endeffectorjnt = rigidBodyJoint('endeffectorjnt','fixed');
%setFixedTransform(endeffectorjnt, trvec2tform([0.3, 0, 0]));
dof = [1 0 0 0 0 0 0]

%Access the rigid bodies joint properties and 
%assign each one its own joint property
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
endeffector.Joint = endeffectorjnt;

%Add each body in sequence
addBody(rbt,body1,'base');
addBody(rbt,body2,'body1');
addBody(rbt,body3,'body2');
addBody(rbt,body4,'body3');
addBody(rbt,endeffector,'body4');



showdetails(rbt)
show(rbt)



%Create a circle that can be traced
t = (0:0.2:10)'; %Time
count = length(t);
center = [0.3 0.4 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

q0 = homeConfiguration(rbt);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', rbt);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'toolTip'


qInitial = q0; % Use home configuration as the initial guess
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
