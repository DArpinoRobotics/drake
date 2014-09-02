function [foot_origin_knots, zmp_knots] = planSwing(biped, stance, swing1, swing2, initial_hold_time)
% Compute a collision-free swing trajectory for a single foot.
if nargin < 5
  initial_hold_time = 0;
end

assert(swing1.frame_id == swing2.frame_id, 'planSwing expects to plan a swing trajectory between two positions of the /same/ foot body')

params = struct(swing2.walking_params);
params = applyDefaults(params, biped.default_walking_params);

DEBUG = true;
ASCENT_ANGLE = pi/3;
DESCENT_ANGLE = -pi/3;
% terrain = biped.getTerrain();
% biped = biped.setTerrain([]);
% biped = biped.compile();
% v = biped.constructVisualizer();
% biped = biped.setTerrain(terrain);
% biped = biped.compile();

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
xy_dist = norm(swing2.pos(1:2) - swing1.pos(1:2));
terrain_slice = double(swing2.terrain_pts);
terrain_slice = [[0;0], terrain_slice, [xy_dist; 0]];
% terrain_slice = [terrain_slice, [0-1, xy_dist+1; 0, 0], [-1, xy_dist + 1; -1, -1]]; % make sure we at least create a step apex 

terrain_pts_in_local = [terrain_slice(1,:); zeros(1, size(terrain_slice, 2)); 
                        terrain_slice(2,:)];

% Extend the terrain slice in the direction perpendicular to the swing
terrain_pts_in_local = [bsxfun(@plus, terrain_pts_in_local, [0;-1;0]), ...
                        bsxfun(@plus, terrain_pts_in_local, [0;1;0])];
                      
% Extend the terrain slice down
terrain_pts_in_local = [terrain_pts_in_local, ...
                        bsxfun(@plus, terrain_pts_in_local, [0;0;-1])];


% Transform to world coordinates
T_local_to_world = [[rotmat(atan2(swing2.pos(2) - swing1.pos(2), swing2.pos(1) - swing1.pos(1))), [0;0];
                     0, 0, 1], swing1.pos(1:3); 
                    0, 0, 0, 1];
terrain_pts_in_world = T_local_to_world * [terrain_pts_in_local; ones(1, size(terrain_pts_in_local, 2))];
terrain_pts_in_world = terrain_pts_in_world(1:3,:);

terrain_hull_distance = [0, cumsum(sqrt(sum(diff(terrain_pts_in_world, 1, 2).^2)))];

% Create terrain geometry.
terrain_geometry = RigidBodyMeshPoints(terrain_pts_in_world);
terrain_geometry.serializeToLCM();
biped = biped.addShapeToBody('world',terrain_geometry);
biped = biped.compile();

% Setup timing
n_t_samples = 9;
t_toe_lift = params.toe_support_frac;
t_swing_rise_end = t_toe_lift + params.swing_rise_frac;
t_heel_land = params.toe_support_frac + 1;
t_swing_fall_begin = t_heel_land - params.swing_fall_frac;
t_final = 1+params.toe_support_frac+params.heel_support_frac;
t = [0; t_toe_lift; t_swing_fall_begin; t_heel_land; t_final;linspace(0,t_final,10)'];
t = unique(t);
t_samples = setdiff(linspace(0,t_final,n_t_samples),t);

% Create posture constraints
xstar = biped.loadFixedPoint();
xstar([1:2,6]) = stance.pos([1:2,6]);

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
stance_body_index = biped.getFrame(stance.frame_id).body_ind;
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
step_height_constraint = MinDistanceConstraint(biped, params.step_height, ...
                            active_collision_options, ...
                            [t_swing_rise_end,t_swing_fall_begin]);


swing_hat = (swing2.pos(1:2) - swing1.pos(1:2)) / norm(swing2.pos(1:2) - swing1.pos(1:2));
collision_plane_info = struct('tspan', {}, 'T', {}, 'distance', {});

% % Keep the foot above its initial plane
collision_plane_info(end+1).tspan = [0, t_swing_rise_end];
collision_plane_info(end).T = T_swing1_sole_to_world;
collision_plane_info(end).distance = -0.005;
% 
% % Keep the foot within its ascent slope
% th1 = atan2(swing_hat(2), swing_hat(1));
% th2 = ASCENT_ANGLE;
% T1 = [eye(3), mean(swing1_toe_points_in_world(1:3,:), 2); 0,0,0,1];
% T2 = [rpy2rotmat([0,0,th1]), [0;0;0]; 0,0,0,1];
% T3 = [rpy2rotmat([0,-th2,0]), [0;0;0]; 0,0,0,1];
% collision_plane_info(end+1).tspan = [0, t_swing_rise_end];
% collision_plane_info(end).T = T1 * T2 * T3;
% collision_plane_info(end).distance = -sqrt(sum(diff(swing1_toe_points_in_world, 1, 2).^2, 1))/2 - 0.005;
% 
% % Keep the foot above its final plane
collision_plane_info(end+1).tspan = [t_swing_fall_begin, t_final];
collision_plane_info(end).T = T_swing2_sole_to_world;
collision_plane_info(end).distance = -0.005;
% 
% Keep the foot within its descent slope
% th1 = atan2(swing_hat(2), swing_hat(1));
% th2 = DESCENT_ANGLE;
% T1 = [eye(3), mean(swing2_heel_points_in_world(1:3,:), 2); 0,0,0,1];
% T2 = [rpy2rotmat([0,0,th1]), [0;0;0]; 0,0,0,1];
% T3 = [rpy2rotmat([0,-th2,0]), [0;0;0]; 0,0,0,1];
% collision_plane_info(end+1).tspan = [t_swing_fall_begin, t_final];
% collision_plane_info(end).T = T1 * T2 * T3;
% collision_plane_info(end).distance = -sqrt(sum(diff(swing2_heel_points_in_world, 1, 2).^2, 1))/2 - 0.005;

% normalized_terrain_hull_t = t_toe_lift + (terrain_hull_distance / terrain_hull_distance(end));
% for j = 1:(length(normalized_terrain_hull_t)-1)
%   tspan = normalized_terrain_hull_t(j:j+1);
%   tspan(1) = max(tspan(1), t_swing_rise_end);
%   tspan(2) = min(tspan(2), t_swing_fall_begin);
%   if tspan(2) <= tspan(1)
%     continue;
%   end
%   th1 = atan2(swing_hat(2), swing_hat(1));
%   th2 = atan2(terrain_pts_in_world(3,j+1)-terrain_pts_in_world(3,j),...
%               swing_hat' * (terrain_pts_in_world(1:2,j+1) - terrain_pts_in_world(1:2,j)));
%   T1 = [eye(3), terrain_pts_in_world(1:3,j); 0,0,0,1];
%   T2 = [rpy2rotmat([0,0,th1]), [0;0;0]; 0,0,0,1];
%   T3 = [rpy2rotmat([0,-th2,0]), [0;0;0]; 0,0,0,1];
%   T = T1 * T2 * T3;
%   collision_plane_info(end+1).tspan = tspan;
%   collision_plane_info(end).T = T;
%   collision_plane_info(end).distance = params.step_height;
% end

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'plane_transform');
for j = 1:length(collision_plane_info)
  T = collision_plane_info(j).T;
  lcmglDrawHT(lcmgl, T);
end

lcmgl.switchBuffers();

swing_plane_constraints = {};
active_collision_pts = [swing_toe_points_in_foot, swing_heel_points_in_foot];
for j = 1:length(collision_plane_info)
  swing_plane_constraints{end+1} = WorldPositionInFrameConstraint(biped,swing_body_index,...
     active_collision_pts, collision_plane_info(j).T,...
     repmat([NaN; NaN; collision_plane_info(j).distance], 1, size(active_collision_pts, 2)),...
     repmat([NaN; NaN; NaN], 1, size(active_collision_pts, 2)),...
     collision_plane_info(j).tspan);
end
                          
% Create lateral constraint on swing foot
lateral_tol = 1e-2; % Distance the sole can move to away from the line 
                    % between step1 and step
swing_lateral_constraint = ...
  WorldPositionInFrameConstraint(biped,swing1.frame_id, ...
    [0;0;0], T_local_to_world, [NaN;-lateral_tol;NaN], [NaN;lateral_tol;NaN]);
  
forward_progress_constraints = { ...
  WorldPositionInFrameConstraint(biped,swing1.frame_id, ...
    [0;0;0], T_local_to_world, [-0.1*xy_dist; NaN; NaN], [0.3*xy_dist; NaN; NaN], [0, t_toe_lift + 1/3]),...
  WorldPositionInFrameConstraint(biped,swing1.frame_id, ...
    [0;0;0], T_local_to_world, [0.9*xy_dist; NaN; NaN], [1.1*xy_dist; NaN; NaN], [t_heel_land - 0.5, t_final]),...
    };
                
constraints = { ...
  upper_body_posture_constraint,...
  stance_foot_position_constraint, ...
  stance_foot_quat_constraint, ...
  swing1_heel_constraint, ...
  swing1_toe_constraint, ...
  swing2_heel_constraint, ...
  swing2_toe_constraint, ...
  swing_lateral_constraint, ...
  swing_plane_constraints{:},...
  step_height_constraint,...
  forward_progress_constraints{:},...
};
% WorldPositionConstraint(biped, swing_body_index, active_collision_pts, repmat([NaN, NaN, 0.06]',1,4), repmat([NaN, NaN, NaN]',1,4), [t_swing_rise_end, t_swing_fall_begin]),... 

q_nom_traj = ConstantTrajectory(xstar(1:biped.getNumPositions()));

ikoptions = IKoptions(biped);
ikoptions = ikoptions.setFixInitialState(false);
ikoptions = ikoptions.setAdditionaltSamples(t_samples);
cost = Point(biped.getStateFrame(), 1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_yaw = 0;
cost = double(cost(1:biped.getNumPositions()));
ikoptions = ikoptions.setQ(diag(cost));
ikoptions = ikoptions.setIterationsLimit(1e6);
[xtraj,info] = inverseKinTraj(biped,t,q_nom_traj,q_nom_traj, ...
                              constraints{:},ikoptions);
info
% v.draw(0, xstar);

if DEBUG
    joint_names = biped.getStateFrame.coordinates(1:biped.getNumPositions());
    joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
    walking_plan = WalkingPlan(xtraj.getBreaks(), xtraj, joint_names);
    lc = lcm.lcm.LCM.getSingleton();
    lc.publish('WALKING_TRAJ_RESPONSE', walking_plan.toLCM());
end
    
% v.playback(xtraj, struct('slider', true));
% keyboard();

num_swing_samples = 50;
foot_origin_knots = struct('t', {}, 'right', {}, 'left', {});
% sample_times = [0, linspace(params.toe_support_frac, 1 + params.toe_support_frac, num_swing_samples), 1 + (params.toe_support_frac + params.heel_support_frac)];
sample_times = linspace(0, 1 + (params.toe_support_frac + params.heel_support_frac), num_swing_samples);
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
    stance_origin_pose = biped.forwardKin(kinsol, stance_body_index, [0;0;0], 1);
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
  foot_origin_knots(j).t = foot_origin_knots(j).t * swing_time + 0.5 * hold_time + initial_hold_time;
end

foot_origin_knots = [foot_origin_knots, foot_origin_knots(end)];
foot_origin_knots(end).t = foot_origin_knots(end-1).t + 0.5 * hold_time;

heel_lift_time = 0.5 * hold_time + initial_hold_time;
toe_lift_time = heel_lift_time + params.toe_support_frac * swing_time;
heel_land_time = toe_lift_time + swing_time;
toe_land_time = heel_land_time + params.heel_support_frac * swing_time;
step_duration = toe_land_time + 0.5 * hold_time;

instep_shift = [0.0;stance.walking_params.drake_instep_shift;0];
zmp1 = shift_step_inward(biped, stance, instep_shift);
zmp2 = mean([stance.pos(1:2), swing2.pos(1:2)], 2);

zmp_knots = struct('t', {}, 'zmp', {}, 'supp', {});
zmp_knots(end+1) = struct('t', heel_lift_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index], {{'heel', 'toe'}, {'toe'}}));
zmp_knots(end+1) = struct('t', toe_lift_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, stance_body_index));
zmp_knots(end+1) = struct('t', heel_land_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index], {{'heel', 'toe'}, {'heel'}}));
zmp_knots(end+1) = struct('t', toe_land_time, 'zmp', zmp1, 'supp', RigidBodySupportState(biped, [stance_body_index, swing_body_index]));
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
