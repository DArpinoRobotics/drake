function [zmptraj, link_constraints, support_times, supports] = planZMPTraj(biped, q0, footsteps, options)
% Plan the trajectories of the ZMP and the feet in order to follow the given footsteps
% @param q0 the initial configuration vector
% @param footsteps a list of Footstep objects
% @option t0 the initial time offset of the trajectories to be generated (default: 0)
% @option first_step_hold_s the number of seconds to wait before lifting the first foot (default: 1)

if nargin < 4; options = struct(); end

if ~isfield(options, 't0'); options.t0 = 0; end
if ~isfield(options, 'first_step_hold_s'); options.first_step_hold_s = 1; end

typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[biped.getNumPositions,1]);

is_right_foot = footsteps(1).frame_id == biped.foot_frame_id.right;

foot0 = feetPosition(biped, q0);
foot0.right(6) = foot0.left(6) + angleDiff(foot0.left(6), foot0.right(6));

steps.right = footsteps([footsteps.frame_id] == biped.foot_frame_id.right);
steps.left = footsteps([footsteps.frame_id] == biped.foot_frame_id.left);
steps.right(1).pos = foot0.right;
steps.left(1).pos = foot0.left;

for f = {'left', 'right'}
  foot = f{1};
  for k = 1:(length(steps.(foot))-1)
    p1 = steps.(foot)(k).pos;
    p2 = steps.(foot)(k+1).pos;
    steps.(foot)(k+1).pos = [p2(1:3); p1(4:6) + angleDiff(p1(4:6), p2(4:6))];
  end
end

function pos = feetCenter(rstep,lstep)
  rcen = rstep.pos;
  lcen = lstep.pos;
  pos = mean([rcen(1:3,:),lcen(1:3,:)],2);
end

rfoot_body_idx = biped.getFrame(biped.foot_frame_id.right).body_ind;
lfoot_body_idx = biped.getFrame(biped.foot_frame_id.left).body_ind;
zmp0 = [];
initial_supports = [];
if steps.right(1).is_in_contact
  z = steps.right(1).pos;
  zmp0(:,end+1) = z(1:2);
  initial_supports(end+1) = rfoot_body_idx;
end
if supp0.left
  z = steps.left(1).pos;
  zmp0(:,end+1) = z(1:2);
  initial_supports(end+1) = lfoot_body_idx;
end
zmp0 = mean(zmp0, 2);
supp0 = RigidBodySupportState(biped, initial_supports);


step_knots = struct('t', options.t0, ...
  'right', steps.right(1).pos,...
  'left', steps.left(1).pos,...
zmp_knots = struct('t', options.t0, 'zmp', zmp0, 'supp', supp0);

istep = struct('right', 1, 'left', 1);
is_first_step = true;

while 1
  if is_right_foot
    sw_foot = 'right'; % moving (swing) foot
    st_foot = 'left'; % stance foot
  else
    sw_foot = 'left';
    st_foot = 'right';
  end
  sw0 = steps.(sw_foot)(istep.(sw_foot));
  sw1 = steps.(sw_foot)(istep.(sw_foot)+1);
  st = steps.(st_foot)(istep.(st_foot));

  if is_first_step
    sw1.walking_params.drake_min_hold_time = options.first_step_hold_s;
    is_first_step = false;
  end
  [new_step_knots, new_zmp_knots] = planSwing(biped, st, sw0, sw1);
  step_knots = [step_knots, new_step_knots];
  zmp_knots = [zmp_knots, new_zmp_knots];

  istep.(sw_foot) = istep.(sw_foot) + 1;

  is_right_foot = ~is_right_foot;
  if istep.left == length(steps.left) && istep.right == length(steps.right)
    break
  end
end

% add a segment at the end to recover
t0 = step_knots(end).t;
step_knots(end+1) = step_knots(end);
step_knots(end).t = t0 + 1.5;
zmpf = mean([steps.right(end).pos(1:2), steps.left(end).pos(1:2)]);
zmp_knots(end+1) =  struct('t', step_knots(end).t, 'zmp', zmpf, 'supp', RigidBodySupportState(biped, [rfoot_body_idx, lfoot_body_idx]));

% Build trajectories
bodytraj = containers.Map('KeyType', 'int32', 'ValueType', 'any');
link_constraints = struct('link_ndx',{}, 'pt', {}, 'traj', {}, 'dtraj', {}, 'ddtraj', {});
for f = {'right', 'left'}
  foot = f{1};
  frame_id = biped.foot_frame_id.(foot);
  body_ind = biped.getFrame(frame_id).body_ind;
  T = biped.getFrame(frame_id).T;
  step_poses = [step_knots.(foot)];
  for j = 1:size(step_poses, 2);
    Tsole = [rpy2rotmat(step_poses(4:6,j)), step_poses(1:3,j); 0 0 0 1];
    Torig = Tsole * inv(T);
    step_poses(:,j) = [Torig(1:3,4); rotmat2rpy(Torig(1:3,1:3))];
    step_poses(step_knots(j).([foot, '_isnan']), j) = nan;
  end
  link_constraints(end+1) = struct('link_ndx', body_ind, 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', PPTrajectory(foh([step_knots.t], step_poses)));
end
zmptraj = PPTrajectory(foh([zmp_knots.t], [zmp_knots.zmp]));

% create support body trajectory
support_times = [zmp_knots.t];
supports = [zmp_knots.supp];

end
