#include "campo_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderCampoPage() {
  String body;
  body.reserve(5200);

  body += "<main class=\"panel\">";
  body += "<div class=\"header\">";
  body += "<div><div class=\"title\">Posicionamento em Campo</div><div class=\"muted\">Campo 2D vista superior em proporcao oficial</div></div>";
  body += HtmlTemplate::backButton("/");
  body += "</div>";

  body += "<div class=\"muted\" id=\"ultraMeta\" style=\"margin-bottom:10px;\">Referencias: campo 182 x 243 cm, circulo central diametro 60 cm, robo diametro 21 cm.</div>";
  body += "<section style=\"display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin:0 0 12px;padding:12px 14px;border:1px solid #4b1a1a;border-radius:12px;background:linear-gradient(180deg,#160b0b,#0f0909);\">";
  body += "<div><div style=\"font-weight:700;\">Alvo por coordenada</div><div class=\"muted\" style=\"font-size:.86rem;\">Informe um alvo no formato x/y para visualizar vetor, angulo e area de tolerancia.</div></div>";
  body += "<button id=\"manualToggle\" type=\"button\" class=\"btn\" aria-pressed=\"false\">Alvo: desligado</button>";
  body += "</section>";

  body += "<div style=\"width:min(94vw,820px);margin:0 auto;\">";
  body += "<svg id=\"campoSvg\" viewBox=\"0 0 1980 2590\" style=\"width:100%;height:auto;display:block;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.35);background:#0a0a0a;\" aria-label=\"Campo RoboCup 2D\">";

  body += "<rect x=\"40\" y=\"40\" width=\"1900\" height=\"2510\" rx=\"36\" fill=\"#0d0d0d\"/>";
  body += "<rect x=\"80\" y=\"80\" width=\"1820\" height=\"2430\" fill=\"#00b53f\"/>";

  body += "<rect x=\"80\" y=\"80\" width=\"1820\" height=\"2430\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"18\"/>";
  body += "<rect x=\"200\" y=\"200\" width=\"1580\" height=\"2190\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"10\" opacity=\"0.85\"/>";

  body += "<circle cx=\"990\" cy=\"1295\" r=\"300\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";

  body += "<path d=\"M 440 200 H 1540 V 300 Q 1540 450 1390 450 H 590 Q 440 450 440 300 Z\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";
  body += "<path d=\"M 590 2140 H 1390 Q 1540 2140 1540 2290 V 2390 H 440 V 2290 Q 440 2140 590 2140 Z\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";

  body += "<rect x=\"700\" y=\"110\" width=\"580\" height=\"60\" fill=\"#ffd21f\" opacity=\"0.95\"/>";
  body += "<rect x=\"700\" y=\"2420\" width=\"580\" height=\"60\" fill=\"#1f5eff\" opacity=\"0.95\"/>";

  body += "<g id=\"robot\" transform=\"translate(990 1540)\">";
  body += "<circle r=\"105\" fill=\"#202020\" stroke=\"#ffffff\" stroke-width=\"8\"/>";
  body += "<circle r=\"8\" fill=\"#ffffff\"/>";
  body += "<line x1=\"0\" y1=\"0\" x2=\"50\" y2=\"-40\" stroke=\"#ff6f3c\" stroke-width=\"9\" stroke-linecap=\"round\"/>";
  body += "</g>";
  body += "<g id=\"targetLayer\" style=\"display:none;\">";
  body += "<rect id=\"toleranceRect\" x=\"0\" y=\"0\" width=\"0\" height=\"0\" fill=\"rgba(180,180,180,0.18)\" stroke=\"#bbbbbb\" stroke-width=\"8\" stroke-dasharray=\"18 12\"/>";
  body += "<line id=\"vectorLine\" x1=\"0\" y1=\"0\" x2=\"0\" y2=\"0\" stroke=\"#d9d9d9\" stroke-width=\"10\" stroke-linecap=\"round\" opacity=\"0.85\"/>";
  body += "<circle id=\"targetPoint\" cx=\"0\" cy=\"0\" r=\"34\" fill=\"#d0d0d0\" stroke=\"#ffffff\" stroke-width=\"8\"/>";
  body += "<text id=\"targetLabel\" x=\"0\" y=\"0\" fill=\"#ffffff\" font-size=\"44\" font-weight=\"800\" text-anchor=\"middle\">A</text>";
  body += "</g>";

  body += "</svg>";
  body += "</div>";

  body += "<div id=\"manualModal\" style=\"position:fixed;inset:0;background:rgba(0,0,0,.68);display:none;align-items:center;justify-content:center;padding:20px;z-index:30;\">";
  body += "<div style=\"width:min(420px,100%);background:linear-gradient(180deg,#171010,#0f0909);border:1px solid #5a1e1e;border-radius:16px;padding:18px;box-shadow:0 18px 40px rgba(0,0,0,.45);\">";
  body += "<div style=\"font-size:1.05rem;font-weight:800;margin-bottom:6px;\">Definir alvo de destino</div>";
  body += "<div class=\"muted\" style=\"margin-bottom:12px;\">Use o formato x/y, por exemplo 39/78.</div>";
  body += "<label style=\"display:block;margin-bottom:10px;\"><span style=\"display:block;margin-bottom:4px;font-weight:700;\">Coordenada alvo</span><input id=\"manualTarget\" type=\"text\" inputmode=\"decimal\" placeholder=\"39/78\" style=\"width:100%;padding:10px 12px;border-radius:10px;border:1px solid #6a2a2a;background:#0a0a0a;color:#f3f3f3;\"></label>";
  body += "<div class=\"muted\" id=\"manualHint\" style=\"margin-bottom:12px;\"></div>";
  body += "<div id=\"manualError\" style=\"display:none;margin-bottom:12px;color:#ff8f8f;font-weight:700;\"></div>";
  body += "<div style=\"display:flex;justify-content:flex-end;gap:10px;flex-wrap:wrap;\">";
  body += "<button id=\"manualCancel\" type=\"button\" style=\"padding:10px 14px;border-radius:10px;border:1px solid #5f2525;background:#181010;color:#f3f3f3;font-weight:700;\">Cancelar</button>";
  body += "<button id=\"manualSave\" type=\"button\" class=\"btn\">Enviar alvo</button>";
  body += "</div>";
  body += "</div>";
  body += "</div>";

  body += "</main>";

  String script;
  script.reserve(13600);
  script += "<script>";
  script += "const FIELD_W=182.0,FIELD_H=243.0,ROBOT_D=21.0,ROBOT_R=ROBOT_D/2.0;";
  script += "const COMP_TOL=22.0,MAX_JUMP=20.0;";
  script += "const GROSS_RESIDUAL=55.0,FREEZE_CONF=0.52;";
  script += "const TAU_FAST=0.18,TAU_SLOW=0.42;";
  script += "const TARGET_TOL=10.0;";
  script += "const SVG_W=1980.0,SVG_H=2590.0,FIELD_SVG_X=80.0,FIELD_SVG_Y=80.0,FIELD_SVG_W=1820.0,FIELD_SVG_H=2430.0;";
  script += "let lastX=FIELD_W/2.0,lastY=FIELD_H/2.0,lastConf=0.0;";
  script += "let lastTickMs=Date.now();";
  script += "let prevEstMs=Date.now(),prevEstX=lastX,prevEstY=lastY,velEstX=0,velEstY=0;let tickBusy=false;";
  script += "let manualMode=false;let targetPos={x:FIELD_W/2.0,y:FIELD_H/2.0};let currentAutoPos={x:FIELD_W/2.0,y:FIELD_H/2.0};";
  script += "let filtU={d:null,e:null,f:null,t:null};let histU={d:[],e:[],f:[],t:[]};";
  script += "const campoSvg=document.getElementById('campoSvg');const robotEl=document.getElementById('robot');const meta=document.getElementById('ultraMeta');";
  script += "const manualToggle=document.getElementById('manualToggle');const manualModal=document.getElementById('manualModal');const manualTarget=document.getElementById('manualTarget');const manualHint=document.getElementById('manualHint');const manualError=document.getElementById('manualError');const manualCancel=document.getElementById('manualCancel');const manualSave=document.getElementById('manualSave');";
  script += "const targetLayer=document.getElementById('targetLayer');const toleranceRect=document.getElementById('toleranceRect');const vectorLine=document.getElementById('vectorLine');const targetPoint=document.getElementById('targetPoint');const targetLabel=document.getElementById('targetLabel');";

  script += "function isValidDist(v){return Number.isFinite(v)&&v>1&&v<350;}";
  script += "function clamp(v,min,max){return Math.max(min,Math.min(max,v));}";
  script += "function median3(arr){const a=arr.slice().sort((x,y)=>x-y);const m=Math.floor(a.length/2);return a[m];}";
  script += "function pushHist(key,v){if(!isValidDist(v))return;histU[key].push(v);if(histU[key].length>3)histU[key].shift();}";
  script += "function robustSensor(key,v){if(!isValidDist(v))return (filtU[key]==null?-1:filtU[key]);pushHist(key,v);let base=v;if(histU[key].length>=3)base=median3(histU[key]);else if(histU[key].length===2)base=0.5*(histU[key][0]+histU[key][1]);if(filtU[key]==null){filtU[key]=base;return base;}const delta=base-filtU[key];const limit=18.0;filtU[key]=filtU[key]+clamp(delta,-limit,limit)*0.62;return filtU[key];}";
  script += "function prefilterUltras(u){return {d:robustSensor('d',u.d),e:robustSensor('e',u.e),f:robustSensor('f',u.f),t:robustSensor('t',u.t)};}";
  script += "function parseCoord(v){const n=Number(v);return Number.isFinite(n)?n:NaN;}";
  script += "function parseTargetPair(text){const raw=String(text||'').trim();const parts=raw.split('/');if(parts.length!==2)return {ok:false};const x=parseCoord(parts[0].replace(',','.'));const y=parseCoord(parts[1].replace(',','.'));if(!Number.isFinite(x)||!Number.isFinite(y))return {ok:false};return {ok:true,x,y};}";
  script += "function coordMinX(){return ROBOT_R;}function coordMaxX(){return FIELD_W-ROBOT_R;}";
  script += "function coordMinY(){return ROBOT_R;}function coordMaxY(){return FIELD_H-ROBOT_R;}";
  script += "function setManualError(msg){manualError.textContent=msg;manualError.style.display=msg?'block':'none';}";
  script += "function updateManualHint(){manualHint.textContent='Faixa valida do centro do robo: X '+coordMinX().toFixed(1)+' a '+coordMaxX().toFixed(1)+' cm | Y '+coordMinY().toFixed(1)+' a '+coordMaxY().toFixed(1)+' cm | tolerancia: +/- '+TARGET_TOL.toFixed(0)+' cm';}";
  script += "function openManualModal(){updateManualHint();setManualError('');manualTarget.value=targetPos.x.toFixed(1)+'/'+targetPos.y.toFixed(1);manualModal.style.display='flex';}";
  script += "function closeManualModal(){manualModal.style.display='none';}";
  script += "function updateManualButton(){manualToggle.textContent=manualMode?'Alvo: ligado':'Alvo: desligado';manualToggle.setAttribute('aria-pressed',manualMode?'true':'false');}";
  script += "function validateManualCoords(x,y){if(!Number.isFinite(x)||!Number.isFinite(y))return 'Digite a coordenada no formato x/y.';if(x<coordMinX()||x>coordMaxX())return 'X fora do campo util.';if(y<coordMinY()||y>coordMaxY())return 'Y fora do campo util.';return ''; }";
  script += "function toSvgX(x){return 80+x*10.0;}function toSvgY(y){return 80+y*10.0;}";
  script += "function n360(a){let x=a;while(x<0)x+=360;while(x>=360)x-=360;return x;}";
  script += "function calcMoveAngle(current,target){return n360(Math.atan2(target.x-current.x, -(target.y-current.y))*180/Math.PI);}";
  script += "function classifyDriveMode(angle){const a=n360(angle);const normal=(a>=315||a<=45)||(a>=135&&a<=225);if(normal)return 'NORMAL';const laterais=(a>45&&a<135)||(a>225&&a<315);if(laterais)return 'LATERAIS';return 'NORMAL';}";
  script += "function targetMetrics(current,target){const dx=target.x-current.x;const dy=target.y-current.y;const dist=Math.sqrt(dx*dx+dy*dy);const angle=Math.atan2(dy,dx)*180/Math.PI;return {dx,dy,dist,angle};}";
  script += "function drawTargetOverlay(current,target){if(!manualMode){targetLayer.style.display='none';return;}targetLayer.style.display='block';const left=clamp(target.x-TARGET_TOL,coordMinX(),coordMaxX());const right=clamp(target.x+TARGET_TOL,coordMinX(),coordMaxX());const top=clamp(target.y-TARGET_TOL,coordMinY(),coordMaxY());const bottom=clamp(target.y+TARGET_TOL,coordMinY(),coordMaxY());const x1=toSvgX(left),y1=toSvgY(top),x2=toSvgX(right),y2=toSvgY(bottom),tx=toSvgX(target.x),ty=toSvgY(target.y),cx=toSvgX(current.x),cy=toSvgY(current.y);toleranceRect.setAttribute('x',x1.toFixed(1));toleranceRect.setAttribute('y',y1.toFixed(1));toleranceRect.setAttribute('width',(x2-x1).toFixed(1));toleranceRect.setAttribute('height',(y2-y1).toFixed(1));vectorLine.setAttribute('x1',cx.toFixed(1));vectorLine.setAttribute('y1',cy.toFixed(1));vectorLine.setAttribute('x2',tx.toFixed(1));vectorLine.setAttribute('y2',ty.toFixed(1));targetPoint.setAttribute('cx',tx.toFixed(1));targetPoint.setAttribute('cy',ty.toFixed(1));targetLabel.setAttribute('x',tx.toFixed(1));targetLabel.setAttribute('y',(ty+14).toFixed(1));}";
  script += "function clientToFieldCoords(clientX,clientY){const rect=campoSvg.getBoundingClientRect();if(rect.width<=0||rect.height<=0)return null;const sx=(clientX-rect.left)*(SVG_W/rect.width);const sy=(clientY-rect.top)*(SVG_H/rect.height);if(sx<FIELD_SVG_X||sx>(FIELD_SVG_X+FIELD_SVG_W)||sy<FIELD_SVG_Y||sy>(FIELD_SVG_Y+FIELD_SVG_H))return null;const x=clamp((sx-FIELD_SVG_X)/10.0,coordMinX(),coordMaxX());const y=clamp((sy-FIELD_SVG_Y)/10.0,coordMinY(),coordMaxY());return {x,y};}";
  script += "async function enviarAlvoHTTPS(x,y,origem){const angMove=calcMoveAngle(currentAutoPos,{x,y});const modo=classifyDriveMode(angMove);const body=new URLSearchParams();body.set('alvo',x.toFixed(1)+'/'+y.toFixed(1));body.set('modo',modo);const r=await fetch('/api/posicionamento',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},body:body.toString()});const j=await r.json().catch(()=>({}));if(!r.ok||!j.ok)throw new Error('rejeitado');targetPos={x,y};manualMode=true;updateManualButton();drawTargetOverlay(currentAutoPos,targetPos);const m=targetMetrics(currentAutoPos,targetPos);meta.textContent='Alvo '+fmt(targetPos.x)+'/'+fmt(targetPos.y)+' cm enviado por '+origem+' | vetor dx:'+fmt(m.dx)+' dy:'+fmt(m.dy)+' | dist:'+fmt(m.dist)+' cm | ang:'+fmt(m.angle)+' deg | angMov:'+fmt(angMove)+' deg | modo:'+modo;}";

  script += "function complementary(prev,meas,conf,dtSec){";
  script += "const tau=(conf>=0.8)?TAU_FAST:TAU_SLOW;";
  script += "let alpha=Math.exp(-dtSec/Math.max(0.02,tau));";
  script += "alpha=clamp(alpha+(1-conf)*0.18,0.08,0.96);";
  script += "return alpha*prev+(1-alpha)*meas;}";

  script += "function estimateAxis(aRaw,bRaw,field,last,dtSec,axis){";
  script += "const hasA=isValidDist(aRaw),hasB=isValidDist(bRaw);";
  script += "const min=ROBOT_R,max=field-ROBOT_R,usable=field-2*ROBOT_R;";
  script += "let meas=last,conf=0.05,state='fallback',residual=999,sumAB=0;";

  script += "if(hasA&&hasB){";
  script += "const directA=clamp(aRaw+ROBOT_R,min,max);";
  script += "const directB=clamp(field-(bRaw+ROBOT_R),min,max);";
  script += "sumAB=Math.max(1.0,aRaw+bRaw);";
  script += "residual=Math.abs((aRaw+bRaw+2*ROBOT_R)-field);";
  script += "if(residual<=COMP_TOL){";
  script += "meas=(directA+directB)*0.5;conf=0.95;state='compativel';";
  script += "}else{";
  script += "const fracA=clamp(aRaw/sumAB,0,1);";
  script += "const fracB=clamp(bRaw/sumAB,0,1);";
  script += "const mapFromA=min+fracA*usable;";
  script += "const mapFromB=min+(1.0-fracB)*usable;";
  script += "meas=0.5*(mapFromA+mapFromB);";
  script += "conf=0.56;state='incompativel_mapeado';";
  script += "}";
  script += "}else if(hasA){";
  script += "meas=clamp(aRaw+ROBOT_R,min,max);conf=0.62;state=axis+'_single_a';";
  script += "}else if(hasB){";
  script += "meas=clamp(field-(bRaw+ROBOT_R),min,max);conf=0.62;state=axis+'_single_b';";
  script += "}";

  script += "const jump=meas-last;";
  script += "const limited=last+clamp(jump,-MAX_JUMP,MAX_JUMP);";
  script += "const filtered=complementary(last,limited,conf,dtSec);";
  script += "return {x:clamp(filtered,min,max),meas:clamp(meas,min,max),conf,state,residual,sum:sumAB};}";

  script += "function estimateFromUltras(u){";
  script += "const d=u.d,e=u.e,f=u.f,t=u.t;";
  script += "const now=Date.now();const dtSec=clamp((now-lastTickMs)/1000.0,0.05,0.8);lastTickMs=now;";

  script += "const rx=estimateAxis(e,d,FIELD_W,lastX,dtSec,'x');";
  script += "const ry=estimateAxis(f,t,FIELD_H,lastY,dtSec,'y');";

  script += "let x=rx.x;";
  script += "let y=ry.x;";
  script += "const grossBad=(rx.residual>GROSS_RESIDUAL&&ry.residual>GROSS_RESIDUAL);";
  script += "const weak=(rx.conf<FREEZE_CONF&&ry.conf<FREEZE_CONF);";
  script += "if(grossBad&&weak){x=lastX;y=lastY;}";

  script += "lastX=x;lastY=y;lastConf=(rx.conf+ry.conf)*0.5;";
  script += "return {x,y,conf:lastConf,xState:rx.state,yState:ry.state,xResidual:rx.residual,yResidual:ry.residual,xSum:rx.sum,ySum:ry.sum,xMeas:rx.meas,yMeas:ry.meas,dtSec,grossBad,weak};}";

  script += "function predictPose(est,idadeMs){";
  script += "const now=Date.now();";
  script += "const dt=clamp((now-prevEstMs)/1000.0,0.03,0.40);";
  script += "const instVx=(est.x-prevEstX)/dt;const instVy=(est.y-prevEstY)/dt;";
  script += "velEstX=(0.65*velEstX)+(0.35*instVx);velEstY=(0.65*velEstY)+(0.35*instVy);";
  script += "const lookAhead=clamp((Number(idadeMs||0)/1000.0)+0.06,0.0,0.45);";
  script += "const peso=clamp((est.conf-0.45)/0.50,0.0,1.0);";
  script += "const px=clamp(est.x+(velEstX*lookAhead*peso),coordMinX(),coordMaxX());";
  script += "const py=clamp(est.y+(velEstY*lookAhead*peso),coordMinY(),coordMaxY());";
  script += "prevEstX=est.x;prevEstY=est.y;prevEstMs=now;";
  script += "return {x:px,y:py,rawX:est.x,rawY:est.y,vx:velEstX,vy:velEstY,lookAhead,peso};}";

  script += "function drawRobot(pos){";
  script += "const sx=80+pos.x*10.0;const sy=80+pos.y*10.0;";
  script += "robotEl.setAttribute('transform','translate('+sx.toFixed(1)+' '+sy.toFixed(1)+')');}";

  script += "function fmt(v){return Number.isFinite(v)?v.toFixed(1):'-';}";

  script += "async function tick(){if(tickBusy)return;tickBusy=true;try{";
  script += "const r=await fetch('/api/ultras',{cache:'no-store'});if(!r.ok)return;const j=await r.json();";
  script += "const uRaw={d:Number(j.uD_x10)/10.0,e:Number(j.uE_x10)/10.0,f:Number(j.uF_x10)/10.0,t:Number(j.uT_x10)/10.0};";
  script += "const u=prefilterUltras(uRaw);";
  script += "const pos=estimateFromUltras(u);const pred=predictPose(pos,Number(j.idade_ms||0));currentAutoPos={x:pred.x,y:pred.y};drawRobot(pred);";
  script += "if(manualMode){const m=targetMetrics(currentAutoPos,targetPos);const angMove=calcMoveAngle(currentAutoPos,targetPos);const modo=classifyDriveMode(angMove);drawTargetOverlay(currentAutoPos,targetPos);meta.textContent='Alvo '+fmt(targetPos.x)+'/'+fmt(targetPos.y)+' cm | tolerancia X:'+fmt(clamp(targetPos.x-TARGET_TOL,coordMinX(),coordMaxX()))+' a '+fmt(clamp(targetPos.x+TARGET_TOL,coordMinX(),coordMaxX()))+' | Y:'+fmt(clamp(targetPos.y-TARGET_TOL,coordMinY(),coordMaxY()))+' a '+fmt(clamp(targetPos.y+TARGET_TOL,coordMinY(),coordMaxY()))+' | vetor dx:'+fmt(m.dx)+' dy:'+fmt(m.dy)+' | dist:'+fmt(m.dist)+' cm | ang:'+fmt(m.angle)+' deg | angMov:'+fmt(angMove)+' deg | modo:'+modo+' | pre X:'+fmt(pred.x)+' Y:'+fmt(pred.y)+' | idade:'+Number(j.idade_ms||0)+' ms';}else{drawTargetOverlay(currentAutoPos,targetPos);meta.textContent='US raw D:'+fmt(uRaw.d)+' E:'+fmt(uRaw.e)+' F:'+fmt(uRaw.f)+' T:'+fmt(uRaw.t)+' | filt D:'+fmt(u.d)+' E:'+fmt(u.e)+' F:'+fmt(u.f)+' T:'+fmt(u.t)+' | pos x:'+fmt(pos.x)+' y:'+fmt(pos.y)+' cm | pre x:'+fmt(pred.x)+' y:'+fmt(pred.y)+' cm | vel x:'+fmt(pred.vx)+' y:'+fmt(pred.vy)+' cm/s | conf:'+fmt(pos.conf*100)+'% | resid X:'+fmt(pos.xResidual)+' Y:'+fmt(pos.yResidual)+' | hold:'+(pos.grossBad&&pos.weak?'SIM':'NAO')+' | dt:'+fmt(pos.dtSec*1000)+' ms | idade:'+Number(j.idade_ms||0)+' ms';}";
  script += "}catch(_){}finally{tickBusy=false;}}";

  script += "manualToggle.addEventListener('click',()=>{if(!manualMode){openManualModal();}else{manualMode=false;updateManualButton();}});";
  script += "manualCancel.addEventListener('click',()=>{closeManualModal();manualMode=false;updateManualButton();});";
  script += "manualSave.addEventListener('click',async ()=>{const parsed=parseTargetPair(manualTarget.value);const x=parsed.x;const y=parsed.y;const err=(!parsed.ok)?'Digite a coordenada no formato x/y.':validateManualCoords(x,y);if(err){setManualError(err);return;}setManualError('');try{await enviarAlvoHTTPS(x,y,'coordenada');closeManualModal();}catch(_){setManualError('Falha ao enviar alvo para o robo.');}});";
  script += "campoSvg.addEventListener('pointerdown',async (ev)=>{if(!manualMode){meta.textContent='Ative Alvo: ligado para selecionar por toque no campo.';return;}const p=clientToFieldCoords(ev.clientX,ev.clientY);if(!p)return;try{await enviarAlvoHTTPS(p.x,p.y,'toque');}catch(_){meta.textContent='Falha ao enviar alvo por toque.';}});";
  script += "manualModal.addEventListener('click',(ev)=>{if(ev.target===manualModal){closeManualModal();manualMode=false;updateManualButton();}});";

  script += "updateManualHint();updateManualButton();tick();setInterval(tick,70);";
  script += "</script>";

  return HtmlTemplate::wrapPage("Posicionamento em Campo", body, "", script);
}

}  // namespace WebPages
