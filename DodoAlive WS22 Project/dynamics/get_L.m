function L = get_L(in1)
%get_L
%    L = get_L(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:05

q1 = in1(3,:);
q2 = in1(4,:);
t2 = q1.*2.0;
t3 = q2.*2.0;
t4 = cos(t2);
t5 = cos(t3);
t6 = sin(t2);
t7 = t2+t3;
t8 = cos(t7);
t9 = sin(t7);
t10 = t6.*5.447575899483789e+21;
t13 = t4.*2.453370079098769e+37;
t14 = t5.*7.383738199534817e+37;
t15 = t5.*2.362796223851142e+39;
t11 = t9.*2.743451687011294e+22;
t16 = t8.*1.235540799533302e+38;
t19 = t15-2.376015052908954e+39;
t12 = -t11;
t17 = -t16;
t20 = 1.0./t19;
t18 = t10+t12;
t21 = t18.*t20.*3.602879701896397e+15;
L = reshape([t20.*(t13+t14+t17-1.730994387001976e+38).*(4.0./5.0),t21,t21,t20.*(t13-t14+t17+1.730994387001976e+38).*(-4.0./5.0)],[2,2]);
