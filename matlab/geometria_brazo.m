function g = geometria_brazo()
% Medidas del brazo MT8005, generado por gen_geometria.py a partir de geometria.json, no editar a mano
% sello 26deb69d
g.h = 118; g.a1 = 29.7; g.a2 = 200; g.a3 = 140; g.Lf = 57.5; g.Lh = 55;   % mm
g.LT = g.Lf + g.Lh;
g.theta_ref = [0, 76.26, -76.26, 90, 0];   % grados
g.lim_min = [-85, -70, -85, -85, -85];
g.lim_max = [85, 70, 85, 85, 85];
g.reposo = [0, 70, 85, 0, 0];
g.zona = struct('x0', 200, 'x1', 300, 'y0', -125, 'y1', 125, 'zmax', 100);
g.caja = struct('x', 250, 'y', 0, 'l', 80, 'h', 80);
end
