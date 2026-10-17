# Genera src/interfaz_gz.h a partir de interfaz/Interfaz_Robot.html (comprimido con gzip)
# PlatformIO lo corre solo antes de compilar (extra_scripts en platformio.ini).
# Tambien se puede correr a mano:  python scripts/gen_interfaz.py
import gzip, os
try:
    Import("env")
    base = env.subst("$PROJECT_DIR")
except Exception:
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # carpeta del proyecto, arriba de scripts/
html = os.path.join(base, "interfaz", "Interfaz_Robot.html")
dest = os.path.join(base, "src", "interfaz_gz.h")
datos = gzip.compress(open(html, "rb").read(), 9, mtime=0)
filas = [", ".join(str(b) for b in datos[i:i + 24]) for i in range(0, len(datos), 24)]
txt = ("// Generado por gen_interfaz.py a partir de interfaz/Interfaz_Robot.html, no editar a mano\n"
       "#pragma once\n#include <stdint.h>\n#include <stddef.h>\n"
       "const size_t INTERFAZ_GZ_LEN = %d;\n"
       "const uint8_t INTERFAZ_GZ[] = {\n%s\n};\n" % (len(datos), ",\n".join(filas)))
viejo = open(dest).read() if os.path.exists(dest) else ""
if txt != viejo:
    open(dest, "w").write(txt)
    print("interfaz_gz.h actualizado, %d bytes comprimidos" % len(datos))
