function out1 = get_pos_com(in1)
%GET_POS_COM
%    OUT1 = GET_POS_COM(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:03

q1 = in1(3,:);
q2 = in1(4,:);
t2 = q1+q2;
out1 = [cos(q1)./1.6e+2+cos(t2)./4.8e+2;sin(q1)./1.6e+2+sin(t2)./4.8e+2];