function out1 = get_J_v_UL(in1)
%get_J_v_UL
%    OUT1 = get_J_v_UL(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:03

q1 = in1(3,:);
out1 = reshape([1.0,0.0,0.0,1.0,sin(q1).*(-1.0./2.0),cos(q1)./2.0,0.0,0.0],[2,4]);
