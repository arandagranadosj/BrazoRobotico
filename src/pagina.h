#pragma once
const char PAGINA[] = R"HTML(<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Brazo MT8005</title>
<style>
body{font-family:system-ui,sans-serif;background:#f4f6f8;color:#1f2933;margin:0;padding:16px;max-width:560px;margin:auto}
h1{font-size:20px;margin:4px 0 12px}
.c{background:#fff;border-radius:10px;padding:12px 14px;margin-bottom:12px;box-shadow:0 1px 3px #0002}
.f{display:flex;justify-content:space-between;font-size:15px;margin-bottom:4px}
.f b{font-variant-numeric:tabular-nums}
.r{font-size:12px;color:#607080}
input[type=range]{width:100%;height:28px}
.b{display:flex;flex-wrap:wrap;gap:8px}
button{flex:1 1 30%;padding:12px 6px;font-size:14px;border:0;border-radius:8px;background:#1f4e8c;color:#fff}
button.g{background:#2e7d32}button.o{background:#c62828}button.s{background:#546e7a}
#st{font-size:13px;color:#607080;margin-top:6px}
</style></head><body>
<h1>Brazo MT8005</h1>
<div class="c"><div class="f"><span>Velocidad</span><b><span id="vv">-</span> °/s</b></div>
<input type="range" id="vel" min="2" max="60" step="1"></div>
<div id="js"></div>
<div class="c b">
<button onclick="cmd('centro')">Todo al centro</button>
<button class="g" onclick="cmd('reposo')">Ir a reposo</button>
<button class="s" onclick="cmd('guardar')">Guardar reposo</button>
<button class="g" onclick="cmd('prender')">Prender</button>
<button class="o" onclick="cmd('soltar')">Soltar</button>
</div><div id="st">Conectando...</div>
<script>
const N=["J1 base","J2 hombro","J3 codo","J4 muñeca","J5 giro"];let arr=[-1,-1,-1,-1,-1,-1],tm={},ini=0;
function q(u){return fetch(u).then(r=>r.json()).catch(e=>null)}
function arma(e){let h="";for(let i=0;i<5;i++){h+=`<div class="c"><div class="f"><span>${N[i]}</span><b><span id="t${i}">0</span>°</b></div>
<input type="range" id="s${i}" min="${e.lo[i]}" max="${e.hi[i]}" step="1" value="${e.m[i]}">
<div class="r">actual <span id="p${i}">0</span>° · reposo ${e.r[i].toFixed(0)}°</div></div>`}
document.getElementById("js").innerHTML=h;
for(let i=0;i<5;i++){const s=document.getElementById("s"+i);s.oninput=()=>{arr[i]=1;document.getElementById("t"+i).textContent=s.value;manda(i,s.value)};
s.onchange=()=>{q(`/m?j=${i}&v=${s.value}`);setTimeout(()=>arr[i]=-1,800)}}}
function manda(i,v){clearTimeout(tm[i]);tm[i]=setTimeout(()=>q(`/m?j=${i}&v=${v}`),80)}
const vel=document.getElementById("vel");vel.oninput=()=>{document.getElementById("vv").textContent=vel.value;arr[5]=1;clearTimeout(tm.v);tm.v=setTimeout(()=>q("/v?v="+vel.value),100)};
vel.onchange=()=>setTimeout(()=>arr[5]=-1,800);
function cmd(c){q("/c?c="+c).then(()=>ini=0)}
function estado(){q("/e").then(e=>{if(!e){document.getElementById("st").textContent="Sin conexión con el brazo";return}
if(!ini){arma(e);ini=1}
for(let i=0;i<5;i++){document.getElementById("p"+i).textContent=e.p[i].toFixed(1);
if(arr[i]<0){document.getElementById("s"+i).value=e.m[i];document.getElementById("t"+i).textContent=e.m[i].toFixed(0)}}
if(arr[5]<0){vel.value=e.v;document.getElementById("vv").textContent=e.v.toFixed(0)}
document.getElementById("st").textContent=(e.on?"Servos con fuerza":"Servos sueltos")+" · posición guardada: "+(e.g?"sí":"guardando...")})}
setInterval(estado,600);estado();
</script></body></html>)HTML";
