
function e = cost_func_44(x)

global gyrA_glob
global gyrC_glob
global dgyrA
global dgyrC
global accA_glob
global accC_glob


o_1 = repmat(x(1:3), [size(gyrA_glob, 1) 1]);
o_2 = repmat(x(4:6), [size(gyrA_glob, 1) 1]);

gamma_1 = cross(gyrA_glob, cross(gyrA_glob, o_1, 2), 2) + cross(dgyrA, o_1, 2);
gamma_2 = cross(gyrC_glob, cross(gyrC_glob, o_2, 2), 2) + cross(dgyrC, o_2, 2);

e = vecnorm(accA_glob - gamma_1, 2, 2) - vecnorm(accC_glob - gamma_2, 2, 2);


end