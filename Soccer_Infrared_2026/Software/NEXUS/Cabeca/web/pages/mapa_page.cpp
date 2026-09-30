#include "mapa_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderMapaPage() {
  String body;
  body.reserve(1600);

  body += "<main class=\"panel\">";
  body += "<div class=\"header\">";
  body += "<div><div class=\"title\">Mapa dos 32 Sensores - Placa Pe</div><div class=\"muted\" id=\"meta\">Aguardando dados...</div></div>";
  body += HtmlTemplate::backButton("/");
  body += "</div>";

  body += "<div class=\"muted\" style=\"margin-bottom:10px;display:flex;gap:12px;flex-wrap:wrap;\">";
  body += "<span>Desativado: cinza</span><span>Piso: vermelho escuro</span><span>Proximo limiar: vermelho medio</span><span>Linha: vermelho intenso</span>";
  body += "</div>";

  body += "<section id=\"map\" style=\"position:relative;width:min(88vw,680px);aspect-ratio:1/1;margin:8px auto 0;border:2px dashed #7a2a2a;border-radius:50%;background:radial-gradient(circle at 50% 50%,#2d1010 0%,#140808 68%,#070404 100%);overflow:hidden;\"></section>";
  body += "<div class=\"muted\" style=\"margin-top:10px;font-size:.88rem;\">Atualizacao em tempo real (~250 ms).</div>";
  body += "</main>";

  String script;
  script.reserve(2900);
  script += "<script>";
  script += "const mapEl=document.getElementById('map');const metaEl=document.getElementById('meta');const sensors=[];";
  script += "function points(){const out=[];const total=32;const radius=42;const cx=50;const cy=50;const rotationOffsetDeg=-90;for(let i=0;i<total;i++){const ang=(i*(360/total)+rotationOffsetDeg)*Math.PI/180;out.push({x:cx+radius*Math.cos(ang),y:cy+radius*Math.sin(ang)});}return out;}";
  script += "function color(v,l){if(v<=0)return '#8f959d';if(v>=l)return '#ff2a2a';if(v>=Math.max(0,l-250))return '#b81f1f';return '#5b1111';}";
  script += "function build(){const p=points();p.forEach((pt,idx)=>{const d=document.createElement('div');d.style.position='absolute';d.style.left=pt.x+'%';d.style.top=pt.y+'%';d.style.width='clamp(24px,3vw,32px)';d.style.height='clamp(24px,3vw,32px)';d.style.borderRadius='50%';d.style.border='2px solid rgba(0,0,0,.28)';d.style.transform='translate(-50%,-50%)';d.style.display='grid';d.style.placeItems='center';d.style.fontSize='10px';d.style.fontWeight='700';d.style.color='#f2f2f2';const n=idx+1;d.textContent=String(n);d.title='S'+n;mapEl.appendChild(d);sensors.push(d);});}";
  script += "function apply(data){const arr=data.sensores||[];const lim=Number(data.limiar||0);for(let i=0;i<sensors.length;i++){const v=Number(arr[i]||0);sensors[i].style.background=color(v,lim);sensors[i].title='S'+(i+1)+' = '+v;}metaEl.textContent='seq: '+data.seq+' | limiar: '+lim+' | idade: '+Number(data.idade_ms||0)+' ms | '+(data.valido?'online':'sem dados');}";
  script += "async function tick(){try{const r=await fetch('/api/map32',{cache:'no-store'});if(!r.ok)return;const d=await r.json();apply(d);}catch(_){}}";
  script += "build();tick();setInterval(tick,250);";
  script += "</script>";

  return HtmlTemplate::wrapPage("Mapa 32 Sensores", body, "", script);
}

}  // namespace WebPages
