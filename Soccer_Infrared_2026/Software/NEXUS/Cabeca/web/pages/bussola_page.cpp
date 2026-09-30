#include "bussola_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderBussolaPage() {
  String body;
  body.reserve(2200);

  body += "<main class=\"panel\">";
  body += "<div class=\"header\">";
  body += "<div><div class=\"title\">Bussola - Radar</div><div class=\"muted\" id=\"meta\">Aguardando leitura...</div></div>";
  body += HtmlTemplate::backButton("/");
  body += "</div>";

  body += "<div style=\"display:grid;gap:12px;grid-template-columns:1fr;\">";
  body += "<div style=\"width:min(92vw,640px);margin:0 auto;\">";
  body += "<canvas id=\"radar\" width=\"640\" height=\"640\" style=\"width:100%;height:auto;display:block;border-radius:14px;background:radial-gradient(circle at 50% 50%,#2a0d0d 0%,#130808 62%,#050404 100%);box-shadow:inset 0 0 30px rgba(0,0,0,.45),0 10px 24px rgba(0,0,0,.22);\"></canvas>";
  body += "</div>";

  body += "<div style=\"display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px;\">";
  body += "<div class=\"card\"><h3>Angulo Atual</h3><p id=\"txtAtual\">-</p></div>";
  body += "<div class=\"card\"><h3>Referencia EEPROM</h3><p id=\"txtRef\">-</p></div>";
  body += "<div class=\"card\"><h3>Erro (Ref - Atual)</h3><p id=\"txtErro\">-</p></div>";
  body += "</div>";
  body += "</div>";

  body += "</main>";

  String script;
  script.reserve(6200);
  script += "<script>";
  script += "const cvs=document.getElementById('radar');const ctx=cvs.getContext('2d');";
  script += "const meta=document.getElementById('meta');const txtAtual=document.getElementById('txtAtual');const txtRef=document.getElementById('txtRef');const txtErro=document.getElementById('txtErro');";
  script += "let st={atual:0,ref:0,erro:0,atualOk:false,refOk:false,idadeA:0,idadeR:0};";

  script += "function n360(a){while(a<0)a+=360;while(a>=360)a-=360;return a;}";
  script += "function toRad(deg){return (deg-90)*Math.PI/180;}";

  script += "function drawGrid(cx,cy,r){";
  script += "ctx.strokeStyle='rgba(255,78,78,0.50)';ctx.lineWidth=2;";
  script += "for(let k=1;k<=4;k++){ctx.beginPath();ctx.arc(cx,cy,r*k/4,0,Math.PI*2);ctx.stroke();}";
  script += "for(let d=0;d<360;d+=30){const a=toRad(d);ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+r*Math.cos(a),cy+r*Math.sin(a));ctx.stroke();}";
  script += "ctx.fillStyle='rgba(255,218,218,0.92)';ctx.font='bold 16px Segoe UI, sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';";
  script += "for(let d=0;d<360;d+=30){const a=toRad(d);const rr=r+20;ctx.fillText(String(d),cx+rr*Math.cos(a),cy+rr*Math.sin(a));}";
  script += "}";

  script += "function drawSector(cx,cy,r,fromDeg,erroDeg){";
  script += "let a0=toRad(fromDeg),a1=toRad(fromDeg+erroDeg);";
  script += "ctx.fillStyle='rgba(255,40,40,0.9)';";
  script += "ctx.beginPath();ctx.moveTo(cx,cy);ctx.arc(cx,cy,r,a0,a1,erroDeg<0);ctx.closePath();ctx.fill();";
  script += "}";

  script += "function drawNeedle(cx,cy,r,deg,color,w){const a=toRad(deg);ctx.strokeStyle=color;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+r*Math.cos(a),cy+r*Math.sin(a));ctx.stroke();}";

  script += "function render(){";
  script += "const w=cvs.width,h=cvs.height,cx=w*0.5,cy=h*0.5,r=Math.min(w,h)*0.38;";
  script += "ctx.clearRect(0,0,w,h);drawGrid(cx,cy,r);";
  script += "if(st.atualOk&&st.refOk){drawSector(cx,cy,r,n360(st.atual),st.erro);}";
  script += "if(st.refOk){drawNeedle(cx,cy,r,n360(st.ref),'#ff8a8a',5);}";
  script += "if(st.atualOk){drawNeedle(cx,cy,r,n360(st.atual),'#ff2a2a',5);}";
  script += "ctx.fillStyle='#ffdcdc';ctx.beginPath();ctx.arc(cx,cy,6,0,Math.PI*2);ctx.fill();";
  script += "}";

  script += "function fmt(v){return Number.isFinite(v)?v.toFixed(1)+' deg':'-';}";

  script += "function apply(){";
  script += "txtAtual.textContent=st.atualOk?fmt(st.atual):'sem dado';";
  script += "txtRef.textContent=st.refOk?fmt(st.ref):'sem referencia';";
  script += "txtErro.textContent=(st.atualOk&&st.refOk)?fmt(st.erro):'sem dado';";
  script += "meta.textContent='Atual: '+(st.atualOk?'OK':'OFF')+' ('+st.idadeA+' ms) | Referencia: '+(st.refOk?'OK':'OFF')+' ('+st.idadeR+' ms)';";
  script += "render();";
  script += "}";

  script += "async function tick(){";
  script += "try{";
  script += "const r=await fetch('/api/bussola',{cache:'no-store'});if(!r.ok)return;const j=await r.json();";
  script += "st.atual=Number(j.atual_deg||0);st.ref=Number(j.referencia_deg||0);st.erro=Number(j.erro_deg||0);";
  script += "st.atualOk=!!j.atual_valido;st.refOk=!!j.referencia_valida;";
  script += "st.idadeA=Number(j.idade_atual_ms||0);st.idadeR=Number(j.idade_ref_ms||0);";
  script += "apply();";
  script += "}catch(_){}}";

  script += "render();tick();setInterval(tick,250);";
  script += "</script>";

  return HtmlTemplate::wrapPage("Bussola", body, "", script);
}

}  // namespace WebPages
