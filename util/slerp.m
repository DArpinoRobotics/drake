function out = slerp(in1,in2,t)
  % Copied from
  % drc/software/perception/matt_sandbox/matlab/multisense_calib by Matthew
  % Antone
in1 = in1/norm(in1);
in2 = in2/norm(in2);
if dot(in1,in2) < 0
  in2 = -in2;
end
angle = acos(dot(in1, in2));
sin_angle = sin(angle);
if (sin_angle==0)
    out = in1;
else
    out = sin((1-t)*angle)/sin_angle*in1 + sin(t*angle)/sin_angle*in2;
end