function [swing_ts, swing_poses, takeoff_time, landing_time] = planSwing(biped, stance, step1, step2)
% Compute a collision-free swing trajectory for a single foot.

assert(step1.frame_id == step2.frame_id, 'planSwing expects to plan a swing trajcectory between two positions of the /same/ foot body')

params = struct(step2.walking_params);
params = applyDefaults(params, biped.default_walking_params);
last_pos = step1.pos;
next_pos = step2.pos;

if params.step_speed < 0
  % negative step speed is an indicator to take a fast, fixed-duration step (e.g. for recovery)
  fixed_duration = -params.step_speed;
  params.step_speed = 1;
else
  fixed_duration = 0;
end

debug = false;

ignore_height = 0.5; % m, height above which we'll assume that our heightmap is giving us bad data (e.g. returns from an object the robot is carrying)

pre_contact_height = 0.005; % height above the ground to aim for when foot is landing
foot_yaw_rate = 0.75; % rad/s

step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));

if fixed_duration
  apex_fracs = [0.15,0.16, 0.84,0.85];
else
  apex_fracs = [0.15, 0.85];
end

next_pos(4:6) = last_pos(4:6) + angleDiff(last_pos(4:6), next_pos(4:6));
apex_pos = interp1([0, 1], [last_pos, next_pos]', apex_fracs)';
apex_pos(3,:) = last_pos(3) + params.step_height + max([next_pos(3) - last_pos(3), 0]);

apex_pos_l = [apex_fracs * step_dist_xy; apex_pos(3,:)];

% The terrain slice is a 2xN matrix, where the first row is distance along the straight line path from swing1 to swing2 and the second row is height above the z position of swing1.
terrain_slice = step2.terrain_pts;
terrain_pts_in_local = [terrain_slice(1,:); zeros(1, size(terrain_slice, 2)); 
                        terrain_slice(2,:)];

% Extend the terrain slice in the direction perpendicular to the swing
terrain_pts_in_local = [bsxfun(@plus, terrain_pts_in_local, [0;-1;0]), ...
                        bsxfun(@plus, terrain_pts_in_local, [0;1;0])];

% Transform to world coordinates
T_local_to_world = [[rotmat(atan2(next_pos(2) - last_pos(2), next_pos(1) - last_pos(1))), [0;0];
                     0, 0, 1], last_pos(1:3); 
                    0, 0, 0, 1];
terrain_pts_in_world = T_local_to_world * [terrain_pts_in_local; ones(1, size(terrain_pts_in_local, 2))];
terrain_pts_in_world = terrain_pts_in_world(1:3,:);


if step_dist_xy > 0.01 && ~isempty(terrain_pts)
  if any(terrain_pts(2,:) > (max([last_pos(3), next_pos(3)]) + ignore_height))
    % If we're getting extremely high terrain heights, then assume it's bad lidar data
    terrain_pts(2,:) = max([last_pos(3), next_pos(3)]);
  end

  %% Expand terrain convex hull by the size of the foot
  expanded_terrain_pts = [[0;last_pos(3)], apex_pos_l, [step_dist_xy; next_pos(3) + pre_contact_height]];
  [contact_length, ~, contact_height] = contactVolume(biped, step1, step2, struct('nom_z_clearance', params.step_height));
  for j = 1:length(terrain_pts(1,:))
    if terrain_pts(2, j) > (j / length(terrain_pts(1,:))) * (next_pos(3) - last_pos(3)) + last_pos(3) + (params.step_height / 2)
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [-contact_length; contact_height];
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [contact_length; contact_height];
    end
  end
  expanded_terrain_pts(1,:) = bsxfun(@max, bsxfun(@min, expanded_terrain_pts(1,:), step_dist_xy), 0);
  expanded_terrain_pts = expanded_terrain_pts(:, convhull(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'simplify', true));
  expanded_terrain_pts = expanded_terrain_pts(:, end:-1:1); % convert counterclockwise to clockwise convex hull

  %% Draw our swing trajectory around the convex hull of the expanded terrain
  traj_pts = expanded_terrain_pts(:, 1:find(expanded_terrain_pts(1,:) >= step_dist_xy, 1, 'first'));

  % add start and end points
  traj_pts = [[0; last_pos(3)], traj_pts, [step_dist_xy; next_pos(3) + pre_contact_height]];
else
  %% Just ignore the terrain and provide an apex pose
  step_dist_xy = 1;
  traj_pts = [[0; last_pos(3)], [apex_fracs; apex_pos(3,:)], [1;next_pos(3) + pre_contact_height]];
end
traj_pts_xyz = [last_pos(1) + (next_pos(1) - last_pos(1)) * traj_pts(1,:) / step_dist_xy;
                last_pos(2) + (next_pos(2) - last_pos(2)) * traj_pts(1,:) / step_dist_xy;
                traj_pts(2,:)];

% Create terrain geometry.
terrain_geometry = RigidBodyMeshPoints(terrain_pts_in_world);
biped = biped.addShapeToBody('world',terrain_geometry);
biped = biped.compile();

% Setup timing
n_t_samples = 10;
t_toe_lift = params.toe_support_frac;
t_heel_land = params.toe_support_frac + 1;
t_final = 1+params.toe_support_frac+params.heel_support_frac;
t = [0;t_toe_lift; t_heel_land; t_final];
t_samples = setdiff(linspace(0,t_final,n_t_samples),t);

% Create posture constraints
xstar = biped.loadFixedPoint();
leg_joint_position_indices = biped.getLegJointPositionIndices();
upper_body_joint_position_indices = ...
  setdiff(1:biped.getNumPositions(),leg_joint_position_indices);
upper_body_posture_constraint = PostureConstraint();
q_upper_body = xstar(upper_body_joint_position_indices);
upper_body_posture_constraint = ...
  upper_body_posture_constraint.setJointLimits( ...
    upper_body_joint_position_indices,q_upper_body,q_upper_body);

% Create stance constraints
stance xyz = stance.pos(1:3);
stance_quat = rpy2quat(stance.pos(4:6));
stance_foot_position_constraint = ...
  WorldPositionConstraint(biped,stance.frame_id,[0;0;0],stance_xyz,stance_xyz);
stance_foot_quat_constraint = ...
  WorldQuaternionConstraint(biped,stance.frame_id,stance_quat,0);

swing_body_index = biped.getFrame(step1.frame_id).body_ind;
swing_toe_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('toe');
swing_heel_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('heel');
T_sole_to_foot = biped.getFrame(step1.frame_id).T;

T_step1_sole_to_world = ...
  [rpy2rotmat(step1.pos(4:6)),step1.pos(1:3); zeros(3,1), 1];
T_step1_foot_to_world = T_step1_sole_to_world/T_sole_to_foot;
step1_heel_points_in_world = T_step1_foot_to_world(1:3,:) * ...
  [swing_toe_points_in_foot,ones(1,size(swing_toe_points_in_foot,2))];
step1_toe_points_in_world = T_step1_foot_to_world(1:3,:) * ...
  [swing_toe_points_in_foot,ones(1,size(swing_toe_points_in_foot,2))];

step1_heel_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_heel_points, ...
                          step1_heel_points_in_world, ...
                          step1_heel_points_in_world, ...
                          [0, 0]);
step1_toe_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_toe_points, ...
                          step1_toe_points_in_world, ...
                          step1_toe_points_in_world, ...
                          [0, t_toe_lift]);

T_step2_sole_to_world = ...
  [rpy2rotmat(step2.pos(4:6)),step2.pos(1:3); zeros(3,1), 1];
T_step2_foot_to_world = T_step2_sole_to_world/T_sole_to_foot;
step2_heel_points_in_world = T_step2_foot_to_world(1:3,:) * ...
  [swing_toe_points_in_foot,ones(1,size(swing_toe_points_in_foot,2))];
step2_toe_points_in_world = T_step2_foot_to_world(1:3,:) * ...
  [swing_toe_points_in_foot,ones(1,size(swing_toe_points_in_foot,2))];

step2_heel_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_heel_points, ...
                          step2_heel_points_in_world, ...
                          step2_heel_points_in_world, ...
                          [t_heel_land,t_final]);
step2_toe_constraint = WorldPositionConstraint(biped,swing_body_index, ...
                          swing_toe_points, ...
                          step2_toe_points_in_world, ...
                          step2_toe_points_in_world, [t_final,t_final]);

% Create collision avoidance constraint
% Only consider swing foot and world
active_collision_options.body_idx = [1,swing_body_index];
min_distance_constraint = MinDistanceConstraint(biped, params.step_height, ...
                            active_collision_options, ...
                            [t_toe_lift,t_heel_land]);
                
constraints = { ...
  stance_foot_position_constraint, ...
  stance_foot_quat_constraint, ...
  step1_heel_constraint, ...
  step1_toe_constraint, ...
  step2_heel_constraint, ...
  step2_toe_constraint, ...
  min_distance_constraint, ...
  };

q_nom_traj = ConstantTrajectory(xstar(1:biped.getNumPositions()));

ikoptions = IKoptions();
ikoptions = ikoptions.setFixInitialState(false);
ikoptions = ikoptions.setAdditionaltSamples(t_samples);
[xtraj,info] = inverseKinTraj(biped,t,q_nom_traj,q_nom_traj, ...
                              constraints,ikoptions);

%% Compute time required for swing from cartesian distance of poses as well as yaw distance
d_dist = sqrt(sum(diff(traj_pts_xyz, 1, 2).^2, 1));
total_dist_traveled = sum(d_dist);
traj_dts = max([d_dist / params.step_speed;
                (d_dist / total_dist_traveled) .* (abs(angleDiff(next_pos(6), last_pos(6))) / foot_yaw_rate)],[],1);
traj_ts = [0, cumsum(traj_dts)] ;

%% Add time to shift weight
if fixed_duration
  hold_time = 0.1;
  traj_ts = traj_ts * ((fixed_duration - 2*hold_time) / traj_ts(end));
  traj_ts = [0, traj_ts+0.5*hold_time, traj_ts(end) + hold_time];
else
  hold_time = traj_ts(end) * params.hold_frac;
  hold_time = max([hold_time, params.drake_min_hold_time]);
  traj_ts = [0, traj_ts + 0.5 * hold_time, traj_ts(end) + hold_time]; % add time for weight shift
end

traj_pts_xyz = [last_pos(1:3), traj_pts_xyz, next_pos(1:3)];
landing_time = traj_ts(end-1);
takeoff_time = traj_ts(2);

%% Interpolate in rpy to constrain the foot orientation. We may set these values to NaN later to free up the foot orientation
rpy_pts = [last_pos(4:6), interp1(traj_ts([2,end-1]), [last_pos(4:6), next_pos(4:6)]', traj_ts(2:end-1))', next_pos(4:6)];

swing_poses.center = [traj_pts_xyz; rpy_pts];

swing_ts = traj_ts;


if debug
  figure(1)
  clf
  hold on
  plot(terrain_pts(1,:), terrain_pts(2,:), 'g.')
  plot(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'ro')

  t = linspace(traj_ts(1), traj_ts(end));
  xyz = step_traj.eval(t);
  plot(sqrt(sum(bsxfun(@minus, xyz(1:2,:), xyz(1:2,1)).^2)), xyz(3,:),'k')
  axis equal
end

instep_shift = [0.0;st.walking_params.drake_instep_shift;0];
zmp1 = shift_step_inward(biped, st, instep_shift);
zmp2 = mean([swing1.pos(1:2), swing2.pos(1:2)]);
stance_body_ind = biped.getFrame(stance.frame_id).body_ind;
swing_body_ind = biped.getFrame(swing1.frame_id).body_ind;

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
