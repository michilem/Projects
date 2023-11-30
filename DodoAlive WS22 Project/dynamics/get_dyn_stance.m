function ode_stance = get_dyn_stance(t,in2,in3)
%GET_DYN_STANCE
%    ODE_STANCE = GET_DYN_STANCE(T,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 14:58:09

dq1 = in2(7,:);
dq2 = in2(8,:);
dx = in2(5,:);
dy = in2(6,:);
q1 = in2(3,:);
q2 = in2(4,:);
u1 = in3(3,:);
u2 = in3(4,:);
ux = in3(1,:);
uy = in3(2,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = q1+q2;
t7 = dq1.^2;
t8 = dq2.^2;
t9 = q1.*2.0;
t10 = q1.*3.0;
t11 = q2.*2.0;
t12 = q2.*3.0;
t26 = -q2;
t28 = -u2;
t118 = ux.*1.657109796825897e+39;
t119 = uy.*1.657109796825897e+39;
t13 = cos(t9);
t14 = cos(t10);
t15 = cos(t11);
t16 = t2.*3.0;
t17 = t2.^2;
t18 = t3.^2;
t19 = sin(t9);
t20 = sin(t10);
t21 = sin(t11);
t22 = t4.*3.0;
t23 = t4.^2;
t24 = cos(t6);
t25 = sin(t6);
t27 = -t11;
t29 = q2+t6;
t30 = t6+t11;
t31 = t6+t9;
t42 = q1+t26;
t44 = t6.*2.0;
t46 = t6.*3.0;
t59 = (dq1.*dq2.*t5)./2.0e+1;
t60 = (dq1.*dq2.*t5)./4.0e+1;
t61 = (t5.*t7)./4.0e+1;
t66 = t2.*7.3575e-1;
t72 = t2.*1.443457723767772e+18;
t73 = t4.*1.443457723767772e+18;
t74 = t3.*2.161727821137838e+18;
t82 = t2.*t4.*8.660746342606634e+17;
t87 = t2.*t3.*-2.161727821137838e+18;
t88 = t3.*t4.*-2.161727821137838e+18;
t122 = dq1.*dx.*t4.*1.240344181241773e+38;
t126 = dq1.*dx.*t2.*2.483176528861196e+38;
t128 = dq1.*dy.*t4.*2.483176528861196e+38;
t129 = dq1.*dy.*t2.*1.240344181241773e+38;
t140 = dq1.*dq2.*t4.*5.927625896572043e+40;
t152 = dq1.*dq2.*t2.*5.927625896572043e+40;
t170 = t2.*u1.*2.369411392817764e+42;
t171 = t2.*u2.*2.369411392817764e+42;
t172 = t4.*u1.*2.369411392817764e+42;
t173 = t4.*u2.*2.369411392817764e+42;
t196 = t2.*t7.*1.188525903273679e+41;
t198 = t4.*t7.*1.188525903273679e+41;
t32 = cos(t29);
t33 = t24.*2.0;
t34 = cos(t30);
t35 = cos(t31);
t36 = t24.^2;
t37 = sin(t29);
t38 = t25.*2.0;
t39 = sin(t30);
t40 = sin(t31);
t41 = t25.^2;
t43 = q1+t27;
t45 = q1+t44;
t47 = cos(t42);
t49 = sin(t42);
t51 = t2+t24;
t52 = t4+t25;
t53 = cos(t44);
t55 = cos(t46);
t56 = sin(t44);
t58 = sin(t46);
t64 = (dq2.*dx.*t24)./2.0e+1;
t65 = (dq2.*dy.*t25)./2.0e+1;
t67 = -t66;
t68 = t24.*2.4525e-1;
t75 = -t72;
t76 = -t73;
t77 = -t74;
t80 = t2.*t74;
t81 = t4.*t74;
t83 = t24.*1.922304455347816e+18;
t84 = t25.*1.922304455347816e+18;
t85 = t2.*t24.*1.351079888211149e+16;
t86 = t4.*t25.*1.351079888211149e+16;
t91 = t23.*t24.*1.351079888211149e+16;
t92 = t17.*t25.*1.351079888211149e+16;
t97 = t3.*t24.*7.205759403792794e+17;
t98 = t3.*t25.*7.205759403792794e+17;
t101 = t24.*t25.*3.844608910695632e+17;
t102 = t2.*t24.*t25.*4.503599627370496e+15;
t103 = t4.*t24.*t25.*4.503599627370496e+15;
t104 = t3.*t4.*t24.*4.323455642275676e+17;
t105 = t2.*t3.*t25.*4.323455642275676e+17;
t112 = t17.*1.950226700065681e+34;
t113 = t23.*1.950226700065681e+34;
t114 = t18.*3.115378115120897e+35;
t117 = t18.*1.557689057560448e+36;
t125 = t2.*t3.*t24.*1.94711132195056e+34;
t127 = t3.*t4.*t25.*1.94711132195056e+34;
t132 = dq1.*dy.*t24.*8.251708936583197e+37;
t133 = dq2.*dy.*t24.*8.251708936583197e+37;
t134 = dq1.*dx.*t25.*8.251708936583197e+37;
t135 = dq2.*dx.*t25.*8.251708936583197e+37;
t136 = t13.*ux.*3.307584483311395e+39;
t137 = t15.*ux.*4.953451203042226e+39;
t138 = t13.*uy.*3.307584483311395e+39;
t139 = t15.*uy.*4.953451203042226e+39;
t141 = dq1.*dy.*t25.*1.653725792071268e+38;
t142 = dq2.*dy.*t25.*1.653725792071268e+38;
t143 = t19.*ux.*3.307584483311395e+39;
t144 = t21.*ux.*4.953451203042226e+39;
t145 = t19.*uy.*3.307584483311395e+39;
t146 = t21.*uy.*4.953451203042226e+39;
t147 = dq1.*dx.*t14.*1.240344181241773e+38;
t148 = dq1.*dy.*t14.*1.240344181241773e+38;
t149 = dq1.*dx.*t20.*1.240344181241773e+38;
t150 = dq1.*dy.*t20.*1.240344181241773e+38;
t151 = t2.*t4.*t24.*t25.*1.2169445762191e+32;
t153 = dq1.*dx.*t24.*1.653725792071268e+38;
t154 = dq2.*dx.*t24.*1.653725792071268e+38;
t167 = t13.*1.925404838076714e+41;
t168 = t15.*5.794757738994925e+41;
t169 = t19.*1.925404838076714e+41;
t195 = t15.*2.362796223851142e+42;
t197 = -t171;
t199 = -t172;
t200 = dq1.*dq2.*t24.*2.968163353803324e+41;
t201 = dq1.*dq2.*t25.*2.968163353803324e+41;
t216 = t24.*u2.*2.379304493406493e+42;
t217 = t25.*u2.*2.379304493406493e+42;
t222 = -t196;
t227 = -t198;
t242 = t7.*t24.*1.780463834343849e+41;
t243 = t8.*t24.*2.373228149228223e+41;
t244 = t7.*t25.*1.780463834343849e+41;
t245 = t8.*t25.*2.373228149228223e+41;
t48 = cos(t43);
t50 = sin(t43);
t54 = cos(t45);
t57 = sin(t45);
t62 = t16+t33;
t63 = t22+t38;
t69 = -t68;
t78 = t36.*4.503599627370496e+15;
t79 = t41.*4.503599627370496e+15;
t93 = t4.*t36.*-4.503599627370496e+15;
t94 = t2.*t41.*-4.503599627370496e+15;
t95 = t4.*t85;
t96 = t2.*t86;
t99 = -t91;
t100 = -t92;
t106 = -t97;
t107 = -t98;
t108 = -t102;
t109 = -t103;
t110 = -t104;
t111 = -t105;
t115 = t36.*8.657289628797069e+33;
t116 = t41.*8.657289628797069e+33;
t120 = t23.*t36.*6.084722881095501e+31;
t121 = t17.*t41.*6.084722881095501e+31;
t130 = -t125;
t131 = -t127;
t155 = -t138;
t156 = -t146;
t157 = dq1.*dx.*t35.*8.268961208278488e+37;
t158 = dq2.*dx.*t35.*8.268961208278488e+37;
t159 = dq1.*dy.*t35.*8.268961208278488e+37;
t160 = dq2.*dy.*t35.*8.268961208278488e+37;
t161 = dq1.*dx.*t40.*8.268961208278488e+37;
t162 = dq2.*dx.*t40.*8.268961208278488e+37;
t163 = dq1.*dy.*t40.*8.268961208278488e+37;
t164 = dq2.*dy.*t40.*8.268961208278488e+37;
t165 = -t148;
t166 = -t150;
t174 = dq1.*dx.*t32.*3.095300541628314e+38;
t175 = dq1.*dx.*t34.*1.238362800760556e+38;
t176 = dq2.*dx.*t34.*1.238362800760556e+38;
t177 = dq1.*dy.*t47.*4.114666799327076e+37;
t178 = dq1.*dy.*t32.*3.095300541628314e+38;
t179 = dq2.*dy.*t47.*4.114666799327076e+37;
t180 = dq1.*dy.*t34.*1.238362800760556e+38;
t182 = dq2.*dy.*t34.*1.238362800760556e+38;
t184 = dq1.*dx.*t49.*4.114666799327076e+37;
t185 = dq1.*dx.*t37.*3.095300541628314e+38;
t186 = dq2.*dx.*t49.*4.114666799327076e+37;
t187 = dq1.*dx.*t39.*1.238362800760556e+38;
t188 = dq2.*dx.*t39.*1.238362800760556e+38;
t189 = dq1.*dy.*t37.*3.095300541628314e+38;
t190 = dq1.*dy.*t39.*1.238362800760556e+38;
t192 = dq2.*dy.*t39.*1.238362800760556e+38;
t194 = -t167;
t202 = t7.*t34.*1.238362800760556e+38;
t203 = dq1.*dx.*t47.*2.065258921588405e+38;
t205 = dq2.*dx.*t47.*2.065258921588405e+38;
t209 = t7.*t39.*1.238362800760556e+38;
t213 = dq1.*dy.*t49.*2.065258921588405e+38;
t215 = dq2.*dy.*t49.*2.065258921588405e+38;
t218 = dq1.*dx.*t55.*8.251708936583197e+37;
t219 = dq2.*dx.*t55.*8.251708936583197e+37;
t220 = dq1.*dy.*t55.*8.251708936583197e+37;
t221 = dq2.*dy.*t55.*8.251708936583197e+37;
t223 = dq1.*dx.*t58.*8.251708936583197e+37;
t224 = dq2.*dx.*t58.*8.251708936583197e+37;
t225 = dq1.*dy.*t58.*8.251708936583197e+37;
t226 = dq2.*dy.*t58.*8.251708936583197e+37;
t228 = -t200;
t229 = -t201;
t231 = dq1.*dq2.*t32.*2.974130616758116e+40;
t232 = dq1.*dq2.*t34.*2.990646163836744e+40;
t235 = t53.*ux.*3.300683574633279e+39;
t236 = t53.*uy.*3.300683574633279e+39;
t237 = dq1.*dq2.*t37.*2.974130616758116e+40;
t238 = dq1.*dq2.*t39.*2.990646163836744e+40;
t240 = t56.*ux.*3.300683574633279e+39;
t241 = t56.*uy.*3.300683574633279e+39;
t246 = -t217;
t255 = t7.*t32.*6.930831116452489e+40;
t256 = t8.*t34.*2.953495279813927e+40;
t257 = dq1.*dq2.*t47.*9.470899631192677e+39;
t261 = t7.*t37.*6.930831116452489e+40;
t262 = t8.*t39.*2.953495279813927e+40;
t264 = dq1.*dq2.*t49.*9.470899631192677e+39;
t268 = -t242;
t269 = -t243;
t270 = -t244;
t271 = -t245;
t274 = t32.*u1.*2.372703126257226e+42;
t275 = t32.*u2.*2.372703126257226e+42;
t276 = t37.*u1.*2.372703126257226e+42;
t277 = t37.*u2.*2.372703126257226e+42;
t278 = t7.*t47.*1.954427102070737e+40;
t281 = t8.*t47.*4.916191343092942e+40;
t283 = t7.*t49.*1.954427102070737e+40;
t286 = t8.*t49.*4.916191343092942e+40;
t288 = t53.*9.696524194737354e+41;
t289 = t56.*9.696524194737354e+41;
t291 = t47.*u2.*2.362796223851142e+42;
t292 = t49.*u2.*2.362796223851142e+42;
t296 = t195-2.376015052908954e+42;
t70 = (dq1.*dx.*t62)./4.0e+1;
t71 = (dq1.*dy.*t63)./4.0e+1;
t89 = t4.*t78;
t90 = t2.*t79;
t123 = -t120;
t124 = -t121;
t181 = -t159;
t183 = -t160;
t191 = -t163;
t193 = -t164;
t204 = dq1.*dx.*t48.*1.857544201140835e+38;
t206 = -t177;
t207 = dq1.*dy.*t48.*1.857544201140835e+38;
t208 = -t179;
t210 = -t184;
t211 = dq1.*dx.*t50.*1.857544201140835e+38;
t212 = -t186;
t214 = dq1.*dy.*t50.*1.857544201140835e+38;
t230 = -t202;
t234 = -t209;
t247 = dq1.*dx.*t54.*1.237756340487479e+38;
t248 = dq1.*dy.*t54.*1.237756340487479e+38;
t249 = -t220;
t250 = -t221;
t251 = dq1.*dx.*t57.*1.237756340487479e+38;
t252 = dq1.*dy.*t57.*1.237756340487479e+38;
t253 = -t225;
t254 = -t226;
t258 = dq1.*dq2.*t48.*2.953495279813927e+40;
t259 = -t231;
t260 = -t232;
t263 = -t236;
t265 = dq1.*dq2.*t50.*2.953495279813927e+40;
t266 = -t237;
t267 = -t238;
t279 = t7.*t48.*2.953495279813927e+40;
t280 = -t255;
t284 = t7.*t50.*2.953495279813927e+40;
t285 = -t261;
t290 = -t274;
t293 = -t277;
t294 = -t289;
t295 = -t291;
t297 = 1.0./t296;
t298 = t72+t94+t103+t106;
t299 = t73+t93+t102+t107;
t300 = t82+t101+t110+t111;
t301 = t77+t78+t79+t85+t86-1.443457723767772e+18;
t233 = -t207;
t239 = -t211;
t272 = -t248;
t273 = -t252;
t282 = -t258;
t287 = -t265;
t302 = t75+t83+t87+t90+t96+t97+t99+t109;
t303 = t76+t84+t88+t89+t95+t98+t100+t108;
t304 = t112+t113+t115+t116+t117+t123+t124+t130+t131+t151-2.774765213505006e+36;
t305 = 1.0./t304;
t306 = t118+t126+t129+t132+t133+t136+t137+t145+t147+t152+t153+t154+t156+t157+t158+t165+t169+t173+t174+t175+t176+t178+t180+t181+t182+t183+t199+t203+t204+t205+t206+t208+t218+t219+t222+t228+t230+t233+t235+t241+t246+t247+t249+t250+t256+t257+t259+t260+t268+t269+t272+t276+t278+t279+t280+t281+t282+t292+t293+t294;
t307 = t119+t122+t128+t134+t135+t139+t140+t141+t142+t143+t144+t149+t155+t161+t162+t166+t168+t170+t185+t187+t188+t189+t190+t191+t192+t193+t194+t197+t210+t212+t213+t214+t215+t216+t223+t224+t227+t229+t234+t239+t240+t251+t253+t254+t262+t263+t264+t266+t267+t270+t271+t273+t275+t283+t284+t285+t286+t287+t288+t290+t295-1.358484394919151e+42;
t308 = t297.*t306;
t313 = t297.*t307;
t309 = t25.*t308;
t310 = t52.*t308;
t312 = t64+t70+t308+ux;
t314 = t24.*t313;
t316 = t51.*t313;
t317 = t65+t71+t313+uy-1.1772e+2;
t311 = -t310;
t315 = -t314;
t318 = t28+t60+t68+t309+t315;
t319 = t59+t61+t67+t69+t311+t316+u1;
et1 = t305.*t312.*(t17.*3.900453400131362e+33+t36.*1.731457925759414e+33+t114-t2.*t3.*t24.*3.894222643901121e+33-5.549530427010012e+35).*(5.0./1.2e+1)+t300.*t305.*t317.*1.876499844737707e+15;
et2 = t299.*t305.*t319.*-1.801439850948198e+17+t303.*t305.*t318.*1.801439850948198e+17;
et3 = t305.*t317.*(t23.*3.900453400131362e+33+t41.*1.731457925759414e+33+t114-t3.*t4.*t25.*3.894222643901121e+33-5.549530427010012e+35).*(5.0./1.2e+1)+t300.*t305.*t312.*1.876499844737707e+15;
et4 = t298.*t305.*t319.*1.801439850948198e+17-t302.*t305.*t318.*1.801439850948198e+17;
mt1 = [dx;dy;dq1;dq2;et1+et2;et3+et4;t305.*t319.*(t78+t79-1.443457723767772e+18).*2.882303761517117e+19-t299.*t305.*t312.*1.801439850948198e+17+t298.*t305.*t317.*1.801439850948198e+17+t301.*t305.*t318.*2.882303761517117e+19];
mt2 = [t305.*t318.*(t17.*2.026619832316723e+16+t23.*2.026619832316723e+16+t36.*2.251799813685248e+15+t41.*2.251799813685248e+15+t77+t85+t86-3.60518554490561e+18).*-5.764607523034235e+19-t303.*t305.*t312.*1.801439850948198e+17+t302.*t305.*t317.*1.801439850948198e+17-t301.*t305.*t319.*2.882303761517117e+19];
ode_stance = [mt1;mt2];
