classdef RigidBodyMeshPoints < RigidBodyMesh
  % RigidBodyMeshPoints   Represents the convex hull of a set of points
  % This class allows for the programatic creation of geometries
  % consisting of the convex hull of a set of points. Visualization is
  % not yet supported for this class.
  %
  % RigidBodyMeshPoints properties:
  %   points - 3 x m array in which the i-th column specifies the
  %            location of the i-th point in body-frame.
  %
  % See also RigidBodyGeometry, RigidBodyMesh
  methods
    function obj = RigidBodyMeshPoints(points)
      obj = obj@RigidBodyMesh('');
      obj.points = points;
    end
    
    function points = getPoints(obj)
      points = obj.points;
    end
    
    function msg = serializeToLCM(obj)
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'terrain points');
      color = [1, .3, .3];
      lcmgl.glColor3f(color(1), color(2), color(3));
      for j = 1:size(obj.points, 2)
        lcmgl.sphere(obj.points(:,j), 0.01, 20, 20);
      end
      lcmgl.switchBuffers();

      % msg = drake.lcmt_viewer_geometry_data();
      % msg.type = msg.BOX;
      % msg.string_data = '';
      % msg.num_float_data = 3;
      % msg.float_data = [0,0,0];
      
      % msg.position = [0;0;0];
      % msg.quaternion = rotmat2quat(eye(3));
      % msg.color = [0,0,0,0];
      msg = [];
      % error('not implemented yet');
    end
  end
  
  properties
    points
  end
  
end
