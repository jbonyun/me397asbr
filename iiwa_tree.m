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

%% Compare FK_space between the built-in kuka and our own
% For a given pose, compares the location of each joint using our own model
% and the built-in model.
% Comparing to the built-in kuka model, we only look at translation,
% because we know and like how we differ in terms of orientation.

test_kuka_examples;
eg_i = 5;
eg_angles = deg2rad(joints(eg_i,:));
%eg_angles = zeros(1,7);

% Mappings of location names between the two models.
joint_pairs = {
    {'L2', 'iiwa_link_1', 1}
    {'L3', 'iiwa_link_2', 2}
    {'L4', 'iiwa_link_3', 3}
    {'L5', 'iiwa_link_4', 4}
    {'L6', 'iiwa_link_5', 5}
    {'L7', 'iiwa_link_6', 6}
    {'L8', 'iiwa_link_7', 7}
    {'Flange', 'iiwa_link_ee', nan}
    };

% Compare frames
for pair_i = 1:numel(joint_pairs)
    fk = FK_toolbox(kuka, eg_angles, joint_pairs{pair_i}{1});
    fkb = FK_toolbox(kuka_b, eg_angles, joint_pairs{pair_i}{2});
    fkus = FK_space(robot, eg_angles, 'JointNum', joint_pairs{pair_i}{3});
    fkb(1:3, 4) = fkb(1:3, 4) * 1000;
    fprintf('\n%s vs %s vs %d\n', joint_pairs{pair_i}{1}, joint_pairs{pair_i}{2}, joint_pairs{pair_i}{3});
    round([fk(1:3,4) fkb(1:3,4) fkus(1:3,4)],8)
    fprintf('Diff norm = %f\n', norm(fk(1:3,4) - fkb(1:3,4)));
    assert(iseye(inv(fkus) * fk));
    fk_a_b_norm(pair_i) = round(norm(fk(1:3, 4) - fkb(1:3, 4)), 8);
end

%% Compare J_space

eg_indices = [2];
for eg_index = 1:numel(eg_indices)
    eg_i = eg_indices(eg_index);
    eg_angles = deg2rad(joints(eg_i,:));
    %for pair_i = 1:numel(joint_pairs)
    for pair_i = 8:8
        Ja = J_toolbox(kuka, eg_angles, joint_pairs{pair_i}{1});
        Jb = J_toolbox(kuka_b, eg_angles, joint_pairs{pair_i}{2});
        Jb = J_m_to_mm(Jb);
        Jc = J_space(robot, eg_angles, 'JointNum', joint_pairs{pair_i}{3});
        %fkb(1:3, 4) = fkb(1:3, 4) * 1000;
        fprintf('\n%s vs %s vs %d\n', joint_pairs{pair_i}{1}, joint_pairs{pair_i}{2}, joint_pairs{pair_i}{3});
        round(Ja-Jb,5)
        J_a_b_norm(pair_i) = round(norm(Ja-Jb),8);
        %assert(J_a_b_norm(pair_i) == fk_a_b_norm(pair_i));
        %assert(all(round(Ja-Jb,8)==0,'all'));
        %round([fk(1:3,4) fkb(1:3,4) fkus(1:3,4)],8)
        %fprintf('Diff norm = %f\n', norm(fk(1:3,4) - fkb(1:3,4)));
        %assert(iseye(inv(fkus) * fk));
    end    
end
%% temp
pair_i = 8;
Ja = round(J_toolbox(kuka, eg_angles, joint_pairs{pair_i}{1}),5)
Jb = J_toolbox(kuka_b, eg_angles, joint_pairs{pair_i}{2});
Jb = round(J_m_to_mm(Jb),5)
Jc = round(J_space(robot, eg_angles, 'JointNum', joint_pairs{pair_i}{3}),5)
Jcbody = round(J_body(robot, eg_angles),5)
%% Functions
% Code in cells above can find this, whether this cell has been run or not.

function ans = FK_toolbox(robot, joint_angles, frame_name)
    config = robot.randomConfiguration();
    ans=num2cell(joint_angles);
    [config.JointPosition]=ans{:};
    ans = robot.getTransform(config, frame_name);
end

function ans = J_toolbox(robot, joint_angles, frame_name)
    config = robot.randomConfiguration();
    ans=num2cell(joint_angles);
    [config.JointPosition]=ans{:};
    ans = robot.geometricJacobian(config, frame_name);
end

function in_mm = frame_m_to_mm(frame_in_m)
    in_mm = frame_in_m;
    in_mm(4:6,:) = frame_in_m(4:6,:)*1000;
end

function in_mm = transform_m_to_mm(trans_in_m)
    in_mm = trans_in_m;
    in_mm(1:3,4) = trans_in_m(1:3,4)*1000;
end

function in_mm = J_m_to_mm(J_in_m)
    in_mm = J_in_m;
    in_mm(4:6,:) = in_mm(4:6,:) * 1000;
end