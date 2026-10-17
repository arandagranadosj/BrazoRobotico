# Copia las medidas de geometria.json a los tres lugares que las usan:
#   src/geometria.h            firmware (limites, reposo, pasada de J1)
#   interfaz/Interfaz_Robot.html  bloque entre "// <geometria>" y "// </geometria>"
#   matlab/geometria_brazo.m   funcion que devuelve las medidas para los scripts de MATLAB
# Cada copia lleva el mismo sello (GEO_HASH). El firmware lo reporta y la interfaz avisa si no coincide con el suyo.
# PlatformIO lo corre solo antes de compilar (extra_scripts en platformio.ini). Tambien a mano:  python scripts/gen_geometria.py
import json, hashlib, os, re
try:
    Import("env")
    base = env.subst("$PROJECT_DIR")
except Exception:
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # carpeta del proyecto, arriba de scripts/
g = json.load(open(os.path.join(base, "geometria.json"), encoding="utf-8"))
g.pop("_nota", None)
canon = json.dumps(g, sort_keys=True, separators=(",", ":"))
hsh = hashlib.sha1(canon.encode()).hexdigest()[:8]

def escribe(ruta, txt):
    viejo = open(ruta, encoding="utf-8").read() if os.path.exists(ruta) else ""
    if txt != viejo:
        os.makedirs(os.path.dirname(ruta), exist_ok=True)
        open(ruta, "w", encoding="utf-8", newline="\n").write(txt)
        print("actualizado", os.path.relpath(ruta, base))

f = lambda v: ", ".join(("%g" % x) for x in v)
escribe(os.path.join(base, "src", "geometria.h"),
    "// Generado por gen_geometria.py a partir de geometria.json, no editar a mano\n#pragma once\n"
    "#define GEO_HASH \"%s\"\n"
    "const float LIM_MIN[5] = { %s };      // grados desde el centro\n"
    "const float LIM_MAX[5] = { %s };\n"
    "const float REPOSO_DEF[5] = { %s };   // reposo: hombro atras y codo doblado\n"
    "const int8_t J1_LADO = %d;             // J1 siempre termina girando hacia este lado\n"
    "const float  J1_PASO = %.1ff;            // pasada de J1, mayor que el juego de sus engranes\n"
    % (hsh, f(g["limite_min_grados"]), f(g["limite_max_grados"]), f(g["reposo_grados"]), g["j1_lado"], g["j1_pasada_grados"]))

html = os.path.join(base, "interfaz", "Interfaz_Robot.html")
t = open(html, encoding="utf-8").read()
bloque = ("// <geometria> generado por gen_geometria.py a partir de geometria.json, no editar a mano\n"
          "const GEO=%s,GEO_HASH=\"%s\";\n// </geometria>" % (canon, hsh))
t2 = re.sub(r"// <geometria>.*?// </geometria>", lambda m: bloque, t, flags=re.S)
if t2 != t:
    open(html, "w", encoding="utf-8", newline="").write(t2)
    print("actualizado interfaz/Interfaz_Robot.html")

e = g["eslabones_mm"]
escribe(os.path.join(base, "matlab", "geometria_brazo.m"),
    "function g = geometria_brazo()\n"
    "%% Medidas del brazo MT8005, generado por gen_geometria.py a partir de geometria.json, no editar a mano\n"
    "%% sello %s\n"
    "g.h = %g; g.a1 = %g; g.a2 = %g; g.a3 = %g; g.Lf = %g; g.Lh = %g;   %% mm\n"
    "g.LT = g.Lf + g.Lh;\n"
    "g.theta_ref = [%s];   %% grados\n"
    "g.lim_min = [%s];\ng.lim_max = [%s];\ng.reposo = [%s];\n"
    "g.zona = struct('x0', %g, 'x1', %g, 'y0', %g, 'y1', %g, 'zmax', %g);\n"
    "g.caja = struct('x', %g, 'y', %g, 'l', %g, 'h', %g);\nend\n"
    % (hsh, e["h"], e["a1"], e["a2"], e["a3"], e["Lf"], e["Lh"], f(g["theta_ref_grados"]), f(g["limite_min_grados"]),
       f(g["limite_max_grados"]), f(g["reposo_grados"]), *[g["zona_mm"][k] for k in ("x0", "x1", "y0", "y1", "zmax")],
       *[g["caja_mm"][k] for k in ("x", "y", "l", "h")]))
