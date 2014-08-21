function [foot_origin_knots, zmp_knots] = planSwing(biped, stance, swing1, swing2)
% Compute a collision-free swing trajectory for a single foot.

assert(swing1.frame_id == swing2.frame_id, 'planSwing expects to plan a swing trajcectory between two positions of the /same/ foot body')

params = struct(swing2.walking_params);
params = applyDefaults(params, biped.default_walking_params);

DEBUG = true;
% v = biped.constructVisualizer();

ignore_height = 0.5; % m, height above which we'll assume that our heightmap is giving us bad data (e.g. returns from an object the robot is carrying)

pre_contact_height = 0.005; % height above the ground to aim for when foot is landing
foot_yaw_rate = 0.75; % rad/s

stance_body_ind = biped.getFrame(stance.frame_id).body_ind;
swing_body_ind = biped.getFrame(swing1.frame_id).body_ind;
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
terrain_slice = swing2.terrain_pts;
xy_dist = norm(swing2.pos(1:2) - swing1.pos(1:2));
terrain_slice = [terrain_slice, [0, xy_dist; 0, 0]]; % make sure we at least create a step apex 

terrain_pts_in_local = [terrain_slice(1,:); zeros(1, size(terrain_slice, 2)); 
                        terrain_slice(2,:)];

% Extend the terrain slice in the direction perpendicular to the swing
terrain_pts_in_local = [bsxfun(@plus, terrain_pts_in_local, [0;-1;0]), ...
                        bsxfun(@plus, terrain_pts_in_local, [0;1;0])];

% Transform to world coordinates
T_local_to_world = [[rotmat(atan2(swing2.pos(2) - swing1.pos(2), swing2.pos(1) - swing1.pos(1))), [0;0];
                     0, 0, 1], swing1.pos(1:3); 
                    0, 0, 0, 1];
terrain_pts_in_world = T_local_to_world * [terrain_pts_in_local; ones(1, size(terrain_pts_in_local, 2))];
terrain_pts_in_world = terrain_pts_in_world(1:3,:);

% Create terrain geometry.
terrain_geometry = RigidBodyMeshPoints(terrain_pts_in_world);
biped = biped.addShapeToBody('world',terrain_geometry);
biped = biped.compile();

% Setup timing
n_t_samples = 10;
t_toe_lift = params.toe_support_frac;
t_heel_land = params.toe_support_frac + 1;
t_final = 1+params.toe_support_frac+params.heel_support_frac;
t = [0;t_toe_lift; t_heel_land; t_final;linspace(0,t_final,10)'];
t = unique(t);
t_samples = setdiff(linspace(0,t_final,n_t_samples),t);

% Create posture constraints
xstar = biped.loadFixedPoint();
leg_joint_position_indices = biped.getLegJointPositionIndices();
upper_body_joint_position_indices = ...
  setdiff(7:biped.getNumPositions(),leg_joint_position_indices)';
upper_body_posture_constraint = PostureConstraint(biped);
q_upper_body = xstar(upper_body_joint_position_indices);
upper_body_posture_constraint = ...
  upper_body_posture_constraint.setJointLimits( ...
    upper_body_joint_position_indices,q_upper_body,q_upper_body);

% Create stance constraints
stance_xyz = stance.pos(1:3);
stance_quat = rpy2quat(stance.pos(4:6));
stance_foot_position_constraint = ...
  WorldPositionConstraint(biped,stance.frame_id,[0;0;0],stance_xyz,stance_xyz);
stance_foot_quat_constraint = ...
  WorldQuatConstraint(biped,stance.frame_id,stance_quat,0);

swing_body_index = biped.getFrame(swing1.frame_id).body_ind;
swing_toe_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('toe');
swing_heel_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('heel');
T_sole_to_foot = biped.getFrame(swing1.frame_id).T;

T_swing1_sole_to_world = ...
  [rpy2rotmat(swing1.pos(4:6)),swing1.pos(1:3); zeros(1, 3), 1];
T_swing1_foot_to_world = T_swing1_sole_to_world/T_sole_to_foot;
swing1_heel_points_in_world = T_swing1_foot_to_world(1:3,:) * ...
  [swing_heel_points_in_foot; ones(1,size(swing_heel_points_in_foot,2))];
swing1_toe_points_in_world = T_swing1_foot_to_world(1:3,:) * ...
  [swing_toe_points_in_foot; ones(1,size(swing_toe_points_in_foot,2))];

swing1_heel_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_heel_points_in_foot, ...
                          swing1_heel_points_in_world, ...
                          swing1_heel_points_in_world, ...
                          [0, 0]);
swing1_toe_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_toe_points_in_foot, ...
                          swing1_toe_points_in_world, ...
                          swing1_toe_points_in_world, ...
                          [0, t_toe_lift]);

T_swing2_sole_to_world = ...
  [rpy2rotmat(swing2.pos(4:6)),swing2.pos(1:3); zeros(1, 3), 1];
T_swing2_foot_to_world = T_swing2_sole_to_world/T_sole_to_foot;
swing2_heel_points_in_world = T_swing2_foot_to_world(1:3,:) * ...
  [swing_heel_points_in_foot; ones(1,size(swing_heel_points_in_foot,2))];
swing2_toe_points_in_world = T_swing2_foot_to_world(1:3,:) * ...
  [swing_toe_points_in_foot; ones(1,size(swing_toe_points_in_foot,2))];

swing2_heel_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_heel_points_in_foot, ...
                          swing2_heel_points_in_world, ...
                          swing2_heel_points_in_world, ...
                          [t_heel_land,t_final]);
swing2_toe_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_toe_points_in_foot, ...
                          swing2_toe_points_in_world, ...
                          swing2_toe_points_in_world, [t_final,t_final]);

% Create collision avoidance constraint
% Only consider swing foot and world
active_collision_options.body_idx = [1,swing_body_index];
params.step_height
min_distance_constraint = MinDistanceConstraint(biped, params.step_height, ...
                            active_collision_options, ...
                            [t_toe_lift+0.1,t_heel_land-0.1]);

% Create lateral constraint on swing foot
lateral_tol = 1e-2; % Distance the sole can move to away from the line 
                    % between step1 and step
swing_lateral_constraint = ...
  WorldPositionInFrameConstraint(biped,swing1.frame_id, ...
    [0;0;0], T_local_to_world, [NaN;-lateral_tol;NaN], [NaN;lateral_tol;NaN]);
                
constraints = { ...
  upper_body_posture_constraint,...
  stance_foot_position_constraint, ...
  stance_foot_quat_constraint, ...
  swing1_heel_constraint, ...
  swing1_toe_constraint, ...
  swing2_heel_constraint, ...
  swing2_toe_constraint, ...
  min_distance_constraint, ...
  swing_lateral_constraint
  };

q_nom_traj = ConstantTrajectory(xstar(1:biped.getNumPositions()));

ikoptions = IKoptions(biped);
ikoptions = ikoptions.setFixInitialState(false);
ikoptions = ikoptions.setAdditionaltSamples(t_samples);
[xtraj,info] = inverseKinTraj(biped,t,q_nom_traj,q_nom_traj, ...
                              constraints{:},ikoptions);
info
% v.playback(xtraj);

num_swing_samples = 10;
foot_origin_knots = struct('t', {}, 'right', {}, 'left', {});
sample_times = [0, linspace(params.toe_support_frac, 1 + params.toe_support_frac, num_swing_samples), 1 + (params.toe_support_frac + params.heel_support_frac)];
cart_dist = 0;
yaw_dist = 0;

for j = 1:length(sample_times)
  x = xtraj.eval(sample_times(j));
  q = x(1:biped.getNumPositions());
%   v.draw(0, q);
  
  % TODO: is there a faster way?
  kinsol = biped.doKinematics(q);
  swing_origin_pose = biped.forwardKin(kinsol, swing_body_index, [0;0;0], 1);
  if j == 1
    stance_origin_pose = biped.forwardKin(kinsol, stance_body_ind, [0;0;0], 1);
  end
  %% Compute time required for swing from cartesian distance of poses as well as yaw distance
  if j > 1 && j < length(sample_times) - 1
    cart_dist = cart_dist + norm(swing_origin_pose(1:3) - foot_origin_knots(end).(swing_foot_name)(1:3));
    yaw_dist = yaw_dist + abs(swing_origin_pose(6) - foot_origin_knots(end).(swing_foot_name)(6));
  end
  foot_origin_knots(end+1).t = sample_times(j);
  foot_origin_knots(end).(stance_foot_name) = stance_origin_pose;
  foot_origin_knots(end).(swing_foot_name) = swing_origin_pose;
end

swing_time = max(cart_dist / params.step_speed, yaw_dist / foot_yaw_rate);
hold_time = max(swing_time * params.hold_frac, params.drake_min_hold_time);

for j = 1:length(foot_origin_knots)
  foot_origin_knots(j).t = foot_origin_knots(j).t * swing_time + 0.5 * hold_time;
end

foot_origin_knots = [foot_origin_knots, foot_origin_knots(end)];
foot_origin_knots(end).t = foot_origin_knots(end-1).t + 0.5 * hold_time;

heel_lift_time = 0.5 * hold_time;
toe_lift_time = heel_lift_time + params.toe_support_frac * swing_time;
heel_land_time = toe_lift_time + swing_time;
toe_land_time = heel_land_time + params.heel_support_frac * swing_time;
step_duration = toe_land_time + 0.5 * hold_time;

instep_shift = [0.0;stance.walking_params.drake_instep_shift;0];
zmp1 = shift_step_inward(biped, stance, instep_shift);
zmp2 = mean([stance.pos(1:2), swing2.pos(1:2)], 2);

zmp_knots = struct('t', {}, 'zmp', {}, 'supp', {});
zmp_knots(end+1) = struct('t', heel_lift_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_ind, swing_body_ind], {{'heel', 'toe'}, {'toe'}}));
zmp_knots(end+1) = struct('t', toe_lift_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, stance_body_ind));
zmp_knots(end+1) = struct('t', heel_land_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_ind, swing_body_ind], {{'heel', 'toe'}, {'heel'}}));
zmp_knots(end+1) = struct('t', toe_land_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_ind, swing_body_ind]));
zmp_knots(end+1) = struct('t', step_duration, 'zmp', zmp2, 'supp', RigidBodySupportState(biped, [stance_body_ind, swing_body_ind]));

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
