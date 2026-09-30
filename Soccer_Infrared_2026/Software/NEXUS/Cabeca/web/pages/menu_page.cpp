#include "menu_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderMenuPage() {
  String body;
  body.reserve(7600);

  body += "<main class=\"panel\">";
  body += "<div class=\"header nexus-head\">";
  body += "<div>";
  body += "<div class=\"title\">SESI ChangeMakers | NEXUS Dashboard</div>";
  body += "<div class=\"muted\">Painel tatico da Cabeca com identidade visual da equipe</div>";
  body += "</div>";
  body += "<span class=\"nexus-badge\">CE 101 AMERICANA-SP</span>";
  body += "</div>";

  body += "<section class=\"team-strip\" aria-label=\"Equipe\">";
  body += "<div class=\"team-name\">SESI ChangeMakers</div>";
  body += "</section>";

  body += "<section class=\"mini-dash\" aria-label=\"Mini dashboard\">";
  body += "<div class=\"dash-cell dash-brand\"><div class=\"dash-k\">NEXUS</div><div class=\"dash-v\">Dashboard</div></div>";
  body += "<div class=\"dash-cell\"><div class=\"dash-k\">Papel</div><div class=\"dash-v\" id=\"dashPapel\">--</div></div>";
  body += "<div class=\"dash-cell\"><div class=\"dash-k\">Loop FPS</div><div class=\"dash-v\" id=\"dashFps\">--</div></div>";
  body += "<div class=\"dash-cell\"><div class=\"dash-k\">Wi-Fi</div><div class=\"dash-v\" id=\"dashWifi\">--</div></div>";
  body += "</section>";

  body += "<div class=\"muted dash-meta\" id=\"dashMeta\">Atualizando status...</div>";

  body += "<div class=\"section-title\">Navegacao</div>";
  body += "<div class=\"muted\" style=\"margin-bottom:10px;\">4 opcoes organizadas em paginas separadas</div>";

  body += "<section class=\"grid cards-grid\">";
  body += "<a class=\"card card-map\" href=\"/mapa\"><h3>Mapa de Sensores</h3><p>Visualizacao dos 32 sensores da placa Pe.</p></a>";
  body += "<a class=\"card card-field\" href=\"/campo\"><h3>Posicionamento em Campo</h3><p>Estimativa 2D com base em ultrassom.</p></a>";
  body += "<a class=\"card card-compass\" href=\"/bussola\"><h3>Bussola</h3><p>Radar 0-360 com atual, referencia e erro angular.</p></a>";
  body += "<a class=\"card card-next\" href=\"/extra\"><h3>Opcao 4</h3><p>Espaco reservado para a proxima funcionalidade.</p></a>";
  body += "</section>";

  body += "<footer class=\"muted nexus-foot\">NEXUS Web Telemetria em tempo real</footer>";

  body += "</main>";

  String extraHead;
  extraHead.reserve(5200);
  extraHead += "<style>";
  extraHead += ":root{--nexus-black:#0a0a0a;--nexus-black-2:#171717;--nexus-red:#d91111;--nexus-red-2:#ff2a2a;--nexus-ink:#f3f3f3;--nexus-muted:#bababa;}";
  extraHead += "body{background:radial-gradient(circle at 18% 10%,#4a0e0e 0%,#180606 26%,#070707 62%,#020202 100%);color:var(--nexus-ink);}";
  extraHead += ".panel{background:linear-gradient(180deg,#111111 0%,#0a0a0a 62%,#110505 100%);border:1px solid #471717;box-shadow:0 20px 34px rgba(0,0,0,.55);overflow:hidden;}";
  extraHead += ".title{color:#ffffff;text-shadow:0 1px 0 rgba(255,42,42,.35);}";
  extraHead += ".muted{color:var(--nexus-muted);}";
  extraHead += ".nexus-head{padding:8px 2px 12px;border-bottom:1px solid #3a1717;margin-bottom:12px;}";
  extraHead += ".nexus-badge{padding:8px 12px;border-radius:999px;background:linear-gradient(135deg,var(--nexus-red),var(--nexus-red-2));color:#ffffff;font-weight:800;font-size:.75rem;letter-spacing:.05em;border:1px solid #ff6666;}";
  extraHead += ".team-strip{margin:10px 0 12px;padding:12px;border:1px solid #4c1a1a;background:linear-gradient(180deg,#1a0b0b,#100707);border-radius:12px;box-shadow:inset 0 1px 0 rgba(255,55,55,.18);}";
  extraHead += ".team-name{font-size:1.2rem;font-weight:900;letter-spacing:.03em;color:#ffffff;text-transform:none;}";
  extraHead += ".section-title{font-size:.95rem;font-weight:800;color:#ffd0d0;letter-spacing:.04em;text-transform:uppercase;}";
  extraHead += ".mini-dash{display:grid;grid-template-columns:1.3fr repeat(3,1fr);gap:10px;margin:10px 0 8px;}";
  extraHead += ".dash-cell{border:1px solid #632020;background:linear-gradient(180deg,#1f1212,#110b0b);border-radius:12px;padding:10px 12px;min-height:64px;box-shadow:0 4px 12px rgba(0,0,0,.35);}";
  extraHead += ".dash-brand{background:linear-gradient(135deg,#2e0d0d,#d51313);border-color:#ff4f4f;color:#ffffff;}";
  extraHead += ".dash-k{font-size:.72rem;font-weight:700;opacity:.9;letter-spacing:.05em;text-transform:uppercase;color:#ffb9b9;}";
  extraHead += ".dash-v{font-size:1.02rem;font-weight:800;margin-top:2px;color:#ffffff;}";
  extraHead += ".dash-brand .dash-k,.dash-brand .dash-v{color:#fff6f6;}";
  extraHead += ".dash-meta{margin:4px 2px 12px;color:#d6a8a8;}";
  extraHead += ".cards-grid .card{border:1px solid #572121;border-radius:14px;transition:transform .16s ease,box-shadow .16s ease,border-color .16s ease;}";
  extraHead += ".cards-grid .card h3{color:#ffffff;}";
  extraHead += ".cards-grid .card p{color:#dfb8b8;}";
  extraHead += ".cards-grid .card:hover{transform:translateY(-2px);box-shadow:0 10px 20px rgba(0,0,0,.35);border-color:#ff4a4a;}";
  extraHead += ".card-map{background:linear-gradient(180deg,#1f1111,#130c0c);}";
  extraHead += ".card-field{background:linear-gradient(180deg,#210f0f,#150a0a);}";
  extraHead += ".card-compass{background:linear-gradient(180deg,#1d1010,#110909);}";
  extraHead += ".card-next{background:linear-gradient(180deg,#1b0f0f,#100808);}";
  extraHead += ".nexus-foot{margin-top:12px;padding-top:8px;border-top:1px dashed #653131;text-align:right;font-weight:600;color:#ffbbbb;}";
  extraHead += "@media (max-width:920px){.mini-dash{grid-template-columns:1fr 1fr;}.dash-brand{grid-column:1/-1;}}";
  extraHead += "@media (max-width:520px){.mini-dash{grid-template-columns:1fr;}.nexus-foot{text-align:left;}}";
  extraHead += "</style>";

  String extraScript;
  extraScript.reserve(1800);
  extraScript += "<script>";
  extraScript += "const elPapel=document.getElementById('dashPapel');";
  extraScript += "const elFps=document.getElementById('dashFps');";
  extraScript += "const elWifi=document.getElementById('dashWifi');";
  extraScript += "const elMeta=document.getElementById('dashMeta');";
  extraScript += "function fmt(v,d=1){return Number.isFinite(v)?Number(v).toFixed(d):'--';}";
  extraScript += "async function tick(){";
  extraScript += "try{";
  extraScript += "const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)return;const j=await r.json();";
  extraScript += "elPapel.textContent=(j.papel||'--');";
  extraScript += "elFps.textContent=fmt(j.loop_fps,1);";
  extraScript += "elWifi.textContent=(Number(j.wifi_dbm||-127))+' dBm';";
  extraScript += "elMeta.textContent='Clientes AP: '+Number(j.ap_clients||0)+' | idade: '+Number(j.idade_ms||0)+' ms';";
  extraScript += "}catch(_){elMeta.textContent='Sem resposta de status';}";
  extraScript += "}";
  extraScript += "tick();setInterval(tick,450);";
  extraScript += "</script>";

  return HtmlTemplate::wrapPage("NEXUS Cabeca - Menu", body, extraHead, extraScript);
}

}  // namespace WebPages
