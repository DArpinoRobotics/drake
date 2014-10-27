classdef ExternalForceTorque < SingletonCoordinateFrame
  
  methods
    function obj=ExternalForceTorque()
      coordinates = {'body_or_frame_id','fx','fy','fz','tx','ty','tz'};
      obj = obj@SingletonCoordinateFrame('ExternalForceTorque',length(coordinates),'f',coordinates);              
    end
  end
end
