function jacobian_matrix = jacob3001_sym(theta1,theta2,theta3,theta4)
%JACOB3001_SYM
%    JACOBIAN_MATRIX = JACOB3001_SYM(THETA1,THETA2,THETA3,THETA4)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    17-Feb-2025 17:59:22

t2 = (theta1.*pi)./1.8e+2;
t3 = (theta4.*pi)./1.8e+2;
t9 = theta3+7.938034472384487e+1;
t10 = theta2-7.938034472384487e+1;
t4 = cos(t2);
t5 = cos(t3);
t6 = sin(t2);
t7 = sin(t3);
t11 = (t9.*pi)./1.8e+2;
t12 = (t10.*pi)./1.8e+2;
t8 = -t6;
t13 = cos(t11);
t14 = sin(t11);
t15 = cos(t12);
t16 = sin(t12);
t17 = (t13.*t15.*pi)./1.8e+2;
t18 = (t14.*t15.*pi)./1.8e+2;
t19 = (t13.*t16.*pi)./1.8e+2;
t20 = (t14.*t16.*pi)./1.8e+2;
t30 = t4.*t14.*t16.*pi.*(-1.0./1.8e+2);
t31 = t6.*t14.*t16.*pi.*(-1.0./1.8e+2);
t32 = t13.*t15.*pi.*3.947042588679004e+1;
t33 = t14.*t16.*pi.*3.947042588679004e+1;
t35 = t4.*t14.*t15.*pi.*3.947042588679004e+1;
t36 = t4.*t13.*t16.*pi.*3.947042588679004e+1;
t37 = t6.*t14.*t15.*pi.*3.947042588679004e+1;
t38 = t6.*t13.*t16.*pi.*3.947042588679004e+1;
t21 = -t20;
t22 = t4.*t17;
t23 = t4.*t18;
t24 = t4.*t19;
t25 = t6.*t17;
t26 = t4.*t20;
t27 = t6.*t18;
t28 = t6.*t19;
t29 = t6.*t20;
t34 = -t32;
t39 = -t35;
t40 = -t36;
t41 = -t37;
t42 = -t38;
t43 = t18+t19;
t44 = t17+t21;
t45 = t23+t24;
t46 = t27+t28;
t47 = t22+t30;
t48 = t25+t31;
t49 = t7.*t43.*7.643256987045182e+3;
t50 = t5.*t44.*7.643256987045182e+3;
t52 = t5.*t45.*7.643256987045182e+3;
t53 = t5.*t46.*7.643256987045182e+3;
t55 = t7.*t47.*7.643256987045182e+3;
t57 = t7.*t48.*7.643256987045182e+3;
t51 = -t50;
t54 = -t52;
t56 = -t53;
t58 = -t55;
t59 = -t57;
mt1 = [t6.*t33-t5.*t48.*7.643256987045182e+3+t7.*t46.*7.643256987045182e+3-t6.*t15.*pi.*4.145381478760124e+1-t6.*t13.*t15.*pi.*3.947042588679004e+1];
mt2 = [t4.*t32+t5.*t47.*7.643256987045182e+3-t7.*t45.*7.643256987045182e+3+t4.*t15.*pi.*4.145381478760124e+1-t4.*t14.*t16.*pi.*3.947042588679004e+1,0.0,0.0,0.0,1.0];
mt3 = [t39+t40+t54+t58-t4.*t16.*pi.*4.145381478760124e+1,t41+t42+t56+t59-t6.*t16.*pi.*4.145381478760124e+1];
mt4 = [t33+t34+t49+t51-t15.*pi.*4.145381478760124e+1,t8,t4,0.0,t39+t40+t54+t58,t41+t42+t56+t59,t33+t34+t49+t51,t8,t4,0.0,t5.*pi.*(t4.*t13.*t16+t4.*t14.*t15).*(-4.246253881691768e+1)-t7.*pi.*(t4.*t13.*t15-t4.*t14.*t16).*4.246253881691768e+1];
mt5 = [t5.*pi.*(t6.*t13.*t16+t6.*t14.*t15).*(-4.246253881691768e+1)-t7.*pi.*(t6.*t13.*t15+t8.*t14.*t16).*4.246253881691768e+1,t5.*pi.*(t13.*t15-t14.*t16).*(-4.246253881691768e+1)+t7.*pi.*(t13.*t16+t14.*t15).*4.246253881691768e+1,t8];
mt6 = [t4,0.0];
jacobian_matrix = reshape([mt1,mt2,mt3,mt4,mt5,mt6],6,4);
