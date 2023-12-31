function J_cq = get_J_cq(in1)
%get_J_cq
%    J_cq = get_J_cq(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:03

q1 = in1(3,:);
q2 = in1(4,:);
t2 = q1+q2;
t3 = cos(t2);
t4 = sin(t2);
t5 = -t4;
J_cq = reshape([0.0,0.0,0.0,0.0,t5-sin(q1),t3+cos(q1),t5,t3],[2,4]);
