function W_y_F = get_c(in1)
%GET_C
%    W_y_F = GET_C(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:02

q1 = in1(3,:);
q2 = in1(4,:);
y = in1(2,:);
t2 = cos(q2);
t3 = sin(q1);
W_y_F = t3./2.0+y+t3.*(t2./2.0+1.0./2.0)+cos(q1).*sin(q2)+(t2.*t3)./2.0;
