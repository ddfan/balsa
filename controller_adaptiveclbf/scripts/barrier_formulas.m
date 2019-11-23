syms z1 z2 z3 z4 z5 v_lim x_pos y_pos radius gamma_p

z = [z1 z2 z3 z4];
%%
h_velocity = sqrt(z3^2+z4^2)*z5 - v_lim;
dh_velocity = simplify(jacobian(h_velocity,z))
d2h_velocity = simplify(hessian(h_velocity,z))

%%
d = sqrt((z1-x_pos)^2 + (z2-y_pos)^2);
h_pos = gamma_p * (d - radius) + (z1-x_pos) / d * z3 + (z2-y_pos) / d * z4;
dh_pos = (jacobian(h_pos,z))
d2h_pos = (hessian(h_pos,z))

    
