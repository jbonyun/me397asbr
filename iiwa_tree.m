%% Build a Robotics Toolbox model of the iiwa using our own values
% The built-in one is a) built-in and not our own work, b) uses different
% measurements for some links, c) uses different orientations for the
% frames at each joint and end effector. So we make our own. Which we can
% then use to run through the Toolbox algorithms to verify our other work.

clear;

% Our robot definition in our format
robot = robot_iiwa();

% For comparison, the built-in model, in toolbox format
kuka_b = loadrobot('kukaIiwa14');


kuka = rigidBodyTree();
link = rigidBody('L1');
jnt = rigidBodyJoint('space_origin', 'fixed');
link.Joint = jnt;
kuka.addBody(link, kuka.BaseName)

prev_joint_home_transform = [eye(3) [0 0 0]'; 0 0 0 1];

for i = 1:robot.dof
    link = rigidBody(sprintf('L%d', i+1));
    jnt = rigidBodyJoint(sprintf('L%d', i), 'revolute');
    joint_home = [eye(3) robot.offset(i,:)'; 0 0 0 1];
    joint_transform = screw2trans(robot.screw(:,i), 0) * joint_home;
    jnt.setFixedTransform(inv(prev_joint_home_transform) * joint_transform);
    prev_joint_home_transform = joint_transform;
    jnt.JointAxis = robot.axes(i, :);
    link.Joint = jnt;
    kuka.addBody(link, sprintf('L%d', i));
end

link = rigidBody('Flange');
jnt = rigidBodyJoint('flange_joint', 'fixed');
jnt.setFixedTransform(trvec2tform((robot.home(1:3,4) - robot.offset(end,:)')'));
link.Joint = jnt;
kuka.addBody(link, sprintf('L%d', i+1))

%% Compare FK between the built-in kuka to our own
% For a given pose, compares the location of each joint using our own model
% and the built-in model.
% Only looks at translation, because we know we treat orientation
% differently.

test_kuka_examples;
eg_i = 9;
eg_angles = deg2rad(joints(eg_i,:));
%eg_angles = zeros(1,7);

% Mappings of location names between the two models.
joint_pairs = {
    {'L2', 'iiwa_link_1'}
    {'L3', 'iiwa_link_2'}
    {'L4', 'iiwa_link_3'}
    {'L5', 'iiwa_link_4'}
    {'L6', 'iiwa_link_5'}
    {'L7', 'iiwa_link_6'}
    {'L8', 'iiwa_link_7'}
    {'Flange', 'iiwa_link_ee'}
    };

% Compare frames
for pair_i = 1:numel(joint_pairs)
    fk = FK_toolbox(kuka, eg_angles, joint_pairs{pair_i}{1});
    fkb = FK_toolbox(kuka_b, eg_angles, joint_pairs{pair_i}{2});
    fkus = FK_space(robot, eg_angles);
    fprintf('\n%s vs %s\n', joint_pairs{pair_i}{1}, joint_pairs{pair_i}{2});
    round([fk(1:3,4) fkb(1:3,4)*1000],8)
    %diff = fk(1:3,4) - fkb(1:3,4)*1000
    fprintf('Diff norm = %f\n', norm(fk(1:3,4) - fkb(1:3,4)*1000));
end


%% Functions
% Code in cells above can find this, whether this cell has been run or not.

function ans = FK_toolbox(robot, joint_angles, frame_name)
    config = robot.randomConfiguration();
    ans=num2cell(joint_angles);
    [config.JointPosition]=ans{:};
    ans = robot.getTransform(config, frame_name);
end

function in_mm = frame_m_to_mm(frame_in_m)
    in_mm = frame_in_m;
    in_mm(4:6,:) = frame_in_m(4:6,:)*1000;
end

function in_mm = transform_m_to_mm(trans_in_m)
    in_mm = trans_in_m;
    in_mm(1:3,4) = trans_in_m(1:3,4)*1000;
end