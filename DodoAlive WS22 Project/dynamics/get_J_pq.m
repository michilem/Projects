function J_pq = get_J_pq(in1)
%get_J_pq
%    J_pq = get_J_pq(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:03

q2 = in1(4,:);
J_pq = reshape([0.0,1.0,sqrt(2.0).*sin(q2).*1.0./sqrt(cos(q2)+1.0).*(-1.0./2.0),1.0./2.0],[2,2]);
