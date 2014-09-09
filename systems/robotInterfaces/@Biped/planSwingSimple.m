function [foot_origin_knots, zmp_knots] = planSwingSimple(biped, stance, swing1, swing2, initial_hold_time)
% Compute a collision-free swing trajectory for a single foot.
if nargin < 5
  initial_hold_time = 0;
end

assert(swing1.frame_id == swing2.frame_id, 'planSwing expects to plan a swing trajectory between two positions of the /same/ foot body')

params = struct(swing2.walking_params);
params = applyDefaults(params, biped.default_walking_params);

DEBUG = false;
TOE_OFF_ANGLE = pi/8;

% ignore_height = 0.5; % m, height above which we'll assume that our heightmap is giving us bad data (e.g. returns from an object the robot is carrying)

% pre_contact_height = 0.005; % height above the ground to aim for when foot is landing
foot_yaw_rate = 0.75; % rad/s

if stance.frame_id == biped.foot_frame_id.right
  stance_foot_name = 'right';
else
  stance_foot_name = 'left';
end
if swing1.frame_id == biped.foot_frame_id.right
  swing_foot_name = 'right';
else
  swing_foot_name = 'left';
end

swing2.pos(4:6) = swing1.pos(4:6) + angleDiff(swing1.pos(4:6), swing2.pos(4:6));

% The terrain slice is a 2xN matrix, where the first row is distance along the straight line path from swing1 to swing2 and the second row is height above the z position of swing1.
terrain_slice = double(swing2.terrain_pts);
xy_dist = norm(swing2.pos(1:2) - swing1.pos(1:2));
terrain_slice = [[0;0], terrain_slice, [xy_dist; 0]];
terrain_pts_in_local = [terrain_slice(1,:); zeros(1, size(terrain_slice, 2)); 
                        terrain_slice(2,:)];

% Transform to world coordinates
T_local_to_world = [[rotmat(atan2(swing2.pos(2) - swing1.pos(2), swing2.pos(1) - swing1.pos(1))), [0;0];
                     0, 0, 1], swing1.pos(1:3); 
                    0, 0, 0, 1];
% terrain_pts_in_world = T_local_to_world * [terrain_pts_in_local; ones(1, size(terrain_pts_in_local, 2))];
% terrain_pts_in_world = terrain_pts_in_world(1:3,:);


% orig_to_sole = biped.getFrame(swing1.frame_id).T(1:3,4);
% sole_to_toe = mean(biped.getBody(swing_body_index).getTerrainContactPoints('toe'), 2) - orig_to_sole;

% Setup timing
w_toe_lift = params.toe_support_frac;
w_swing_rise_end = w_toe_lift + params.swing_rise_frac;
w_heel_land = params.toe_support_frac + 1;
w_swing_fall_begin = w_heel_land - params.swing_fall_frac;
w_final = 1+params.toe_support_frac+params.heel_support_frac;

% Create posture constraint
xstar = biped.loadFixedPoint();
xstar([1:2,6]) = stance.pos([1:2,6]);

joint_position_indices = [7:biped.getNumPositions()]';
posture_constraint = PostureConstraint(biped);
q_joints = xstar(joint_position_indices);
posture_constraint = posture_constraint.setJointLimits(joint_position_indices, q_joints, q_joints);

swing_body_index = biped.getFrame(swing1.frame_id).body_ind;
stance_body_index = biped.getFrame(stance.frame_id).body_ind;
swing_toe_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('toe');
swing_heel_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('heel');
T_sole_to_foot = biped.getFrame(swing1.frame_id).T;

T_swing1_sole_to_world = ...
  [rpy2rotmat(swing1.pos(4:6)),swing1.pos(1:3); zeros(1, 3), 1];
T_swing1_foot_to_world = T_swing1_sole_to_world/T_sole_to_foot;
swing1_heel_points_in_world = T_swing1_foot_to_world(1:3,:) * ...
  [swing_heel_points_in_foot; ones(1,size(swing_heel_points_in_foot,2))];
swing1_toe_points_in_world = T_swing1_foot_to_world * ...
  [swing_toe_points_in_foot; ones(1,size(swing_toe_points_in_foot,2))];
swing1_toe_points_in_local = T_local_to_world \ swing1_toe_points_in_world;
swing1_toe_points_in_world = swing1_toe_points_in_world(1:3,:);

swing1_heel_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_heel_points_in_foot, ...
                          swing1_heel_points_in_world, ...
                          swing1_heel_points_in_world);
swing1_toe_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_toe_points_in_foot, ...
                          swing1_toe_points_in_world, ...
                          swing1_toe_points_in_world);

T_swing2_sole_to_world = ...
  [rpy2rotmat(swing2.pos(4:6)),swing2.pos(1:3); zeros(1, 3), 1];
T_swing2_foot_to_world = T_swing2_sole_to_world/T_sole_to_foot;
swing2_heel_points_in_world = T_swing2_foot_to_world(1:3,:) * ...
  [swing_heel_points_in_foot; ones(1,size(swing_heel_points_in_foot,2))];
swing2_toe_points_in_world = T_swing2_foot_to_world * ...
  [swing_toe_points_in_foot; ones(1,size(swing_toe_points_in_foot,2))];
swing2_toe_points_in_local = T_local_to_world \ swing2_toe_points_in_world;
swing2_toe_points_in_world = swing2_toe_points_in_world(1:3,:);

stance_toe_points_in_foot = biped.getBody(stance_body_index).getTerrainContactPoints('toe');
T_stance_sole_to_world = ...
  [rpy2rotmat(stance.pos(4:6)),stance.pos(1:3); zeros(1, 3), 1];
T_stance_foot_to_world = T_stance_sole_to_world/biped.getFrame(stance.frame_id).T;
stance_toe_points_in_world = T_stance_foot_to_world(1:3,:) * ...
  [stance_toe_points_in_foot; ones(1, size(stance_toe_points_in_foot, 2))];

swing2_heel_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_heel_points_in_foot, ...
                          swing2_heel_points_in_world, ...
                          swing2_heel_points_in_world);
swing2_toe_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_toe_points_in_foot, ...
                          swing2_toe_points_in_world, ...
                          swing2_toe_points_in_world);


toe1 = mean(swing1_toe_points_in_world, 2);
toe2= mean(swing2_toe_points_in_world, 2);
T_toetoe_to_world = [[rotmat(atan2(toe2(2) - toe1(2), toe2(1) - toe1(1))), [0;0];
                     0, 0, 1], toe1(1:3); 
                    0, 0, 0, 1];

% Create lateral constraint on swing foot
lateral_tol = 1e-6; % Distance the sole can move to away from the line 
                    % between step1 and step
swing_lateral_constraint = ...
  WorldPositionInFrameConstraint(biped,swing_body_index,...
    mean(swing_toe_points_in_foot, 2), ...
    T_toetoe_to_world, [NaN;-lateral_tol;NaN], [NaN;lateral_tol;NaN]);

quat_swing1 = rpy2quat(swing1.pos(4:6));
quat_toe_off = rotmat2quat(quat2rotmat(quat_swing1) * rpy2rotmat([0;TOE_OFF_ANGLE;0]));
quat_swing2 = rpy2quat(swing2.pos(4:6));

% assumes that the toe and heel points are all coplanar with the sole
foot_length = norm(mean(swing1_toe_points_in_world, 2) - mean(swing1_heel_points_in_world, 2));

cost = Point(biped.getStateFrame(),1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_roll = 0;
cost.base_pitch = 0;
cost.base_yaw = 0;
ikoptions = IKoptions(biped);
ikoptions = ikoptions.setQ(diag(cost(1:biped.getNumPositions())));

full_IK_calls = 0;
q_latest = xstar(1:biped.getNumPositions());

if DEBUG
  v = biped.constructVisualizer();
  v.draw(0, q_latest);
end

% basic_constraints = {posture_constraint, swing_lateral_constraint};
basic_constraints = {posture_constraint};

quat_tol = 1e-6;

T = biped.getFrame(stance.frame_id).T;
stance_sole = [rpy2rotmat(stance.pos(4:6)), stance.pos(1:3); 0 0 0 1];
stance_origin = stance_sole * inv(T);
stance_origin_pose = [stance_origin(1:3,4); rotmat2rpy(stance_origin(1:3,1:3))];

T = biped.getFrame(stance.frame_id).T;
swing1_sole = [rpy2rotmat(swing1.pos(4:6)), swing1.pos(1:3); 0 0 0 1];
swing1_origin = swing1_sole * inv(T);
swing1_origin_pose = [swing1_origin(1:3,4); rotmat2rpy(swing1_origin(1:3,1:3))];

swing_poses = [swing1_origin_pose];


foot_origin_knots = struct('t', initial_hold_time + 0.25 * params.drake_min_hold_time, ...
                           swing_foot_name, swing1_origin_pose, ...
                           stance_foot_name, stance_origin_pose);

heel_lift_time = foot_origin_knots(end).t;

function update_foot_knots(constraints, min_dt)
  if nargin < 2
    min_dt = 0;
  end
  constraint_ptrs = {};
  for k = 1:length(constraints)
    constraint_ptrs{end+1} = constraints{k}.mex_ptr;
  end
  full_IK_calls = full_IK_calls + 1;
  [q_latest, info] = inverseKin(biped,q_latest,q_latest,constraint_ptrs{:},ikoptions);
  % info
  if DEBUG
    v.draw(0, q_latest);
  end
  kinsol = biped.doKinematics(q_latest);
  swing_poses(:,end+1) = biped.forwardKin(kinsol, swing_body_index, [0;0;0], 1);
  cartesian_distance = norm(swing_poses(1:3,end) - swing_poses(1:3,end-1));
  yaw_distance = abs(swing_poses(6,end) - swing_poses(6,end-1));
  dt = max(cartesian_distance / params.step_speed, yaw_distance / foot_yaw_rate);
  dt = max(dt, min_dt);
  if dt > 0
    foot_origin_knots(end+1).(swing_foot_name) = swing_poses(:,end);
    foot_origin_knots(end).(stance_foot_name) = stance_origin_pose;
    foot_origin_knots(end).t = foot_origin_knots(end-1).t + dt;
  end
end

n_ws = 5;
ws = linspace(0, w_toe_lift, n_ws);
for j = 1:length(ws)
  w = ws(j);
  quat_des = slerp(quat_swing1, quat_toe_off, w / w_toe_lift);
  constraints = [basic_constraints, {WorldQuatConstraint(biped,swing_body_index,quat_des,quat_tol),...
                 swing1_toe_constraint}];
  update_foot_knots(constraints, 0.25 * params.drake_min_hold_time / n_ws);
end
toe_lift_time = foot_origin_knots(end).t;

ws = linspace(w_toe_lift, w_swing_rise_end, 5);
for j = 1:length(ws)
  w = ws(j);
  quat_des = slerp(quat_toe_off, quat_swing2, (w - w_toe_lift) / (w_swing_fall_begin - w_toe_lift));
  toe_pos_in_local = interp1([w_toe_lift, w_heel_land], [mean(swing1_toe_points_in_local(1,:)), mean(swing2_toe_points_in_local(1,:))], w);
  i0 = find((toe_pos_in_local(1) - foot_length) > terrain_pts_in_local(1,:), 1, 'last');
  if isempty(i0)
    i0 = 1;
  end
  i1 = find(toe_pos_in_local(1) < terrain_pts_in_local(1,:), 1, 'first');
  if isempty(i1)
    i1 = size(terrain_pts_in_local, 2);
  end
  max_terrain_ht = max(terrain_pts_in_local(3,i0:i1));
  toe_ht_in_local = max_terrain_ht + interp1([w_toe_lift, w_swing_rise_end], [0, params.step_height], w);
  constraints = [basic_constraints, {swing_lateral_constraint}, ...
                 {WorldPositionInFrameConstraint(biped,swing_body_index,...
                    mean(swing_toe_points_in_foot, 2), T_local_to_world, [toe_pos_in_local(1); NaN; toe_ht_in_local], [toe_pos_in_local(1); NaN; toe_ht_in_local]),...
                 WorldQuatConstraint(biped,swing_body_index,quat_des,quat_tol)}]; 
  update_foot_knots(constraints);
end

ws = linspace(w_swing_rise_end, w_swing_fall_begin, 10);
for j = 1:length(ws)
  w = ws(j);
  quat_des = slerp(quat_toe_off, quat_swing2, (w - w_toe_lift) / (w_swing_fall_begin - w_toe_lift));
  toe_pos_in_local = interp1([w_toe_lift, w_heel_land], [mean(swing1_toe_points_in_local(1,:)), mean(swing2_toe_points_in_local(1,:))], w);
  i0 = find((toe_pos_in_local(1) - foot_length) > terrain_pts_in_local(1,:), 1, 'last');
  if isempty(i0)
    i0 = 1;
  end
  i1 = find(toe_pos_in_local(1) < terrain_pts_in_local(1,:), 1, 'first');
  if isempty(i1)
    i1 = size(terrain_pts_in_local, 2);
  end
  max_terrain_ht = max(terrain_pts_in_local(3,i0:i1));
  toe_ht_in_local = max_terrain_ht + params.step_height;
  constraints = [basic_constraints, {swing_lateral_constraint}, ...
                 {WorldPositionInFrameConstraint(biped,swing_body_index,...
                    mean(swing_toe_points_in_foot, 2), T_local_to_world, [toe_pos_in_local(1); NaN; toe_ht_in_local], [toe_pos_in_local(1); NaN; toe_ht_in_local]),...
                 WorldQuatConstraint(biped,swing_body_index,quat_des,quat_tol)}]; 
  update_foot_knots(constraints);
end

ws = linspace(w_swing_fall_begin, w_heel_land, 5);
for j = 1:length(ws)
  w = ws(j);
  quat_des = quat_swing2;
  toe_pos_in_local = interp1([w_toe_lift, w_heel_land], [mean(swing1_toe_points_in_local(1,:)), mean(swing2_toe_points_in_local(1,:))], w);
  i0 = find((toe_pos_in_local(1) - foot_length) > terrain_pts_in_local(1,:), 1, 'last');
  if isempty(i0)
    i0 = 1;
  end
  i1 = find(toe_pos_in_local(1) < terrain_pts_in_local(1,:), 1, 'first');
  if isempty(i1)
    i1 = size(terrain_pts_in_local, 2);
  end
  max_terrain_ht = max(terrain_pts_in_local(3,i0:i1));
  terrain_ht_above_goal = swing1.pos(3) + max_terrain_ht - mean(swing2_toe_points_in_world(3,:));
  toe_ht_in_local = max_terrain_ht + interp1([w_swing_fall_begin, w_heel_land], [params.step_height, -terrain_ht_above_goal], w);
  constraints = [basic_constraints, {swing_lateral_constraint}, ...
                 {WorldPositionInFrameConstraint(biped,swing_body_index,...
                    mean(swing_toe_points_in_foot, 2), T_local_to_world, [toe_pos_in_local(1); NaN; toe_ht_in_local], [toe_pos_in_local(1); NaN; toe_ht_in_local]),...
                 WorldQuatConstraint(biped,swing_body_index,quat_des,quat_tol)}]; 
  update_foot_knots(constraints);
end

heel_land_time = foot_origin_knots(end).t;

full_IK_calls

foot_origin_knots = [foot_origin_knots, foot_origin_knots(end)];
foot_origin_knots(end).t = foot_origin_knots(end-1).t + 0.5 * params.drake_min_hold_time;

step_duration = foot_origin_knots(end).t;

instep_shift = [0.0;stance.walking_params.drake_instep_shift;0];
zmp0 = mean([stance.pos(1:2), swing1.pos(1:2)], 2);
zmp0_toe = mean([stance.pos(1:2), mean(swing1_toe_points_in_world(1:2,:), 2)], 2);
zmp1 = shift_step_inward(biped, stance, instep_shift);
zmp2 = mean([stance.pos(1:2), swing2.pos(1:2)], 2);
zmp2_toe = mean([mean(stance_toe_points_in_world(1:2,:), 2), swing2.pos(1:2)], 2);

zmp_knots = struct('t', {}, 'zmp', {}, 'supp', {});
% zmp_knots(end+1) = struct('t', initial_hold_time, 'zmp', zmp0, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index], {{'heel', 'toe'}, {'toe'}}));
zmp_knots(end+1) = struct('t', heel_lift_time, 'zmp', zmp0, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index], {{'heel', 'toe'}, {'toe'}}));
zmp_knots(end+1) = struct('t', toe_lift_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, stance_body_index));
zmp_knots(end+1) = struct('t', heel_land_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index], {{'heel', 'toe'}, {'heel', 'toe'}}));
zmp_knots(end+1) = struct('t', step_duration, 'zmp', zmp2, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index]));
end

function pos = shift_step_inward(biped, step, instep_shift)
  if step.frame_id == biped.foot_frame_id.left
    instep_shift = [1;-1;1].*instep_shift;
  end
  pos_center = step.pos;
  R = rpy2rotmat(pos_center(4:6));
  shift = R*instep_shift;
  pos = pos_center(1:2) + shift(1:2);
end
