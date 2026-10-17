# Brazo Robótico MT8005

![PlatformIO](https://img.shields.io/badge/PlatformIO-renesas--ra-orange?logo=platformio)
![Arduino](https://img.shields.io/badge/Arduino-UNO%20R4%20WiFi-00878F?logo=arduino)
![C++](https://img.shields.io/badge/C%2B%2B-firmware-00599C?logo=cplusplus)
![HTML](https://img.shields.io/badge/HTML-interfaz%20web-E34F26?logo=html5)

Brazo robótico de 5 grados de libertad controlado con un Arduino UNO R4 WiFi y una PCA9685. Incluye el firmware, una interfaz web que planea y ejecuta tareas de recoger y colocar objetos, y las medidas del brazo en un solo archivo compartido por el firmware, la interfaz y MATLAB.

Proyecto del curso **MT8005 Robótica**, Instituto Tecnológico de Costa Rica, II semestre 2026. Profesor: Juan Carlos Brenes Torres.

## Características

- **Control por WiFi o USB.** El Arduino crea su propia red y sirve la interfaz. Por USB se usa Web Serial desde Chrome o Edge.
- **Tareas completas.** Se colocan los objetos en el mapa de la mesa, se elige el agarre y la altura de cada uno, y el brazo los lleva a la caja. Antes de empezar la interfaz simula la tarea y avisa si algún punto queda fuera de alcance o choca con la caja.
- **Trayectorias en lotes.** Por WiFi los puntos llegan en lotes a un búfer del firmware, que los reproduce a tiempo constante aunque la red tarde.
- **Pausa, paso a paso y vueltas continuas**, con modo rápido que acorta las subidas cuando no hace falta la altura completa.
- **Memoria de posición en la EEPROM.** Al prender, el brazo no salta porque recibe la misma posición en la que quedó.
- **Calibración por junta** (offset, escala y juego de engranes) y centro ajustable de cada servo, guardados en el Arduino para que cualquier compu que se conecte la reciba igual.
- **J1 sin juego.** La base siempre termina girando hacia el mismo lado, con una pasada cuando hace falta.
- **Descanso automático.** Si queda quieto en reposo 3 minutos suelta los servos para que no calienten.
- **Vista 3D y vista lateral** del brazo en vivo, modo técnico para ver juntas y datos de cada punto, y una guía corta para quien la usa por primera vez.
- **Herramienta neumática** con relé (preparada en el firmware, inactiva hasta conectar la bomba).

## Hardware

| Componente | Detalle |
| --- | --- |
| Controlador | Arduino UNO R4 WiFi (Renesas RA4M1) |
| Driver de servos | PCA9685 por I2C, dirección `0x40`, 50 Hz |
| Servos | 6 canales: J1 base, J2 hombro, J3a y J3b codo (dos MG996R en espejo), J4 muñeca, J5 giro |
| Herramienta | Bomba de vacío con relé en el pin 6 (activo en bajo) y válvula de alivio opcional |

| Canal PCA9685 | 0 | 1 | 2 | 3 | 4 | 5 |
| --- | --- | --- | --- | --- | --- | --- |
| Junta | J1 | J2 | J3a | J3b | J4 | J5 |

## Estructura

```
Brazo_Robotico/
├── src/
│   ├── main.cpp               firmware
│   └── pagina.h               página sencilla de sliders servida en /
├── interfaz/
│   ├── Interfaz_Robot.html    interfaz principal (la sirve el Arduino en /robot)
│   └── Control_Brazo.html     control por sliders desde la compu por USB
├── scripts/
│   ├── gen_geometria.py       copia geometria.json al firmware, la interfaz y MATLAB
│   └── gen_interfaz.py        comprime la interfaz para guardarla en el Arduino
├── matlab/
│   └── geometria_brazo.m      medidas del brazo para los scripts de MATLAB (generado)
├── .claude/                   configuración de Claude Code para este proyecto
├── geometria.json             medidas del brazo, fuente única
└── platformio.ini
```

`src/geometria.h` y `src/interfaz_gz.h` no están en el repositorio. Los scripts los crean solos antes de cada compilación.

## Compilar y subir

Requisitos: [VS Code](https://code.visualstudio.com/) con la extensión [PlatformIO IDE](https://platformio.org/install/ide?install=vscode). Al abrir la carpeta, VS Code la recomienda sola.

1. Clonar el repositorio y abrir la carpeta en VS Code.
2. Conectar el UNO R4 WiFi por USB-C (cable de datos).
3. Cerrar cualquier programa que use el puerto (la interfaz por USB o el monitor serial).
4. Compilar y subir con el botón **Upload** de PlatformIO, o desde la terminal:

```bash
pio run -e uno_r4_wifi -t upload
```

Para ver el arranque: `pio device monitor` (115200 baudios).

## Usar la interfaz

**Por WiFi**

1. Conectar la compu o el celular a la red `Brazo_Robotico` (clave `robotica123`).
2. Abrir [http://192.168.4.1/robot](http://192.168.4.1/robot).

En Windows, para que se conecte sola a esa red:

```bat
netsh wlan set profileorder name="Brazo_Robotico" interface="Wi-Fi" priority=1
```

**Por USB**

Abrir `interfaz/Interfaz_Robot.html` en Chrome o Edge y elegir USB en el menú **Conectar**. La interfaz encuentra el puerto sola si ya se le dio permiso antes.

**Flujo de una tarea**

1. **Objetos.** Agregar los objetos en el mapa de la mesa (tecla A).
2. **Agarre y altura.** Elegir cómo agarra cada objeto y su altura.
3. **Velocidad y vueltas.** Elegir la velocidad, si repite las vueltas y si usa el modo rápido.
4. **Iniciar proceso** (Enter). **Detener** con Esc y **Reposo** con R.

## Medidas del brazo

Todas las medidas están en `geometria.json`: eslabones, ángulos de referencia, límites de cada junta, reposo, pasada de J1, zona de trabajo y caja. Se cambian solo ahí. Al compilar, `scripts/gen_geometria.py` las copia a:

- `src/geometria.h` para el firmware
- el bloque `// <geometria>` de `interfaz/Interfaz_Robot.html`
- `matlab/geometria_brazo.m`

Las tres copias llevan el mismo sello. Si el firmware y la interfaz no coinciden, la interfaz avisa que hay que volver a subir el firmware.

Para regenerarlas sin compilar:

```bash
python scripts/gen_geometria.py
```

## Comandos por el monitor serial

A 115200 baudios, escribir el comando y Enter. Por WiFi los mismos comandos llegan a `/k?t=comando`.

| Comando | Acción |
| --- | --- |
| `j1 30` | mueve J1 a 30 grados (igual `j2` a `j5`) |
| `v 15` | velocidad de 15 grados/s |
| `c` | centro |
| `r` | reposo |
| `g` | guarda la posición actual como reposo |
| `e` / `x` | prende o suelta los servos |
| `?` | estado |
| `h1 off esc hol` | calibración de J1 en grados: offset, escala y juego de engranes (igual `h2` a `h5`) |
| `t3 1512` | centro del canal 3 en µs, de 1300 a 1700 |
| `p q1 q2 q3 q4 q5` | punto de trayectoria |
| `F` | detiene y vacía los puntos pendientes |
| `aire 1` / `aire 0` | la herramienta agarra o suelta |


## Autor

**Jorge Aranda Granados**, Instituto Tecnológico de Costa Rica.
