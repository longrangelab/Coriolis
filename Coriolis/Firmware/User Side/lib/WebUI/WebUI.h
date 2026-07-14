#ifndef WEBUI_H
#define WEBUI_H

/**
 * WebUI.h  --  phone dashboard hosted by the receiver
 * ===================================================
 * WiFi AP + web server. Phone joins the AP and opens http://192.168.4.1.
 * Tabs: AVG, each node, and SOLVER (ballistic calculator).
 *
 * The solver math lives in Ballistics.h (kept separate). This file only serves
 * the page and forwards /solve requests to a provided callback.
 *
 * FLASH: WiFi + web server is large -> use "Huge APP" partition if it overflows.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

static const char WEBUI_PAGE[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Wind Station</title>
<style>
  :root{--bg:#10151b;--panel:#1a212b;--line:#2b3543;--ink:#f2f6fa;--dim:#8ea1b5;
    --fire:#ffb000;--wind:#4cc2ff;--good:#39d98a;--warnY:#ffd23f;--warn:#ff6b6b;
    --mono:ui-monospace,"SF Mono",Menlo,Consolas,monospace}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);
    font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;-webkit-text-size-adjust:100%}
  .wrap{max-width:640px;margin:0 auto;padding:14px}
  header{display:flex;align-items:center;justify-content:space-between;
    padding:6px 2px 12px;border-bottom:1px solid var(--line)}
  .brand{font-weight:800;letter-spacing:.14em;font-size:.95rem}
  .status{font-family:var(--mono);font-size:.8rem;color:var(--dim)}
  .status.live{color:var(--good)}.status.off{color:var(--warn)}
  .tabs{display:flex;gap:8px;overflow-x:auto;padding:12px 0;scrollbar-width:none}
  .tabs::-webkit-scrollbar{display:none}
  .tab{flex:0 0 auto;min-width:58px;padding:12px 16px;border:1px solid var(--line);
    background:var(--panel);color:var(--dim);border-radius:12px;font-weight:700;font-size:1rem;cursor:pointer}
  .tab.on{background:var(--fire);color:#1a1200;border-color:var(--fire)}
  .hero{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:18px;margin-top:6px}
  .heroTop{display:flex;justify-content:space-between;align-items:baseline}
  .heroLbl{color:var(--dim);font-size:.85rem;letter-spacing:.1em;text-transform:uppercase}
  .spd{font-family:var(--mono);font-size:4.4rem;line-height:.95;font-weight:700}
  .unit{font-size:1.3rem;color:var(--dim);margin-left:8px}
  .rel{font-family:var(--mono);font-size:2.6rem;font-weight:700}
  .relSub{color:var(--dim);font-family:var(--mono);font-size:1rem}
  .card{background:var(--bg);border:1px solid var(--line);border-radius:12px;padding:12px}
  .row{display:flex;gap:10px;margin-top:12px}
  .row .card{flex:1;text-align:center}
  .k{color:var(--dim);font-size:.72rem;letter-spacing:.08em;text-transform:uppercase}
  .v{font-family:var(--mono);font-size:1.5rem;font-weight:700;margin-top:2px}
  h2{font-size:.8rem;letter-spacing:.16em;text-transform:uppercase;color:var(--dim);margin:26px 2px 10px;font-weight:800}
  .wheelWrap{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:16px}
  svg{display:block;width:100%;max-width:340px;margin:0 auto}
  .compass-ring{fill:none;stroke:var(--line);stroke-width:2}
  .tick{stroke:var(--dim);stroke-width:1}
  .card-n{fill:var(--ink);font-family:var(--mono);font-size:13px;font-weight:700}
  .readouts{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}
  .big{font-family:var(--mono);font-size:1.9rem;font-weight:700}
  .ctrl{margin-top:16px}
  .ctrl label{display:flex;justify-content:space-between;color:var(--dim);font-size:.85rem;margin-bottom:6px}
  .azVal{font-family:var(--mono);color:var(--fire);font-size:1.4rem;font-weight:700}
  input[type=range]{width:100%;height:38px;accent-color:var(--fire)}
  .stepRow{display:flex;gap:8px;margin-top:8px}
  .step{flex:1;padding:14px;background:var(--bg);border:1px solid var(--line);border-radius:10px;
    color:var(--ink);font-family:var(--mono);font-size:1.1rem;font-weight:700}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .grid .card{text-align:left}
  .legend{display:flex;gap:16px;justify-content:center;margin-top:8px;font-size:.8rem;color:var(--dim)}
  .dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;vertical-align:middle}
  footer{margin:22px 2px 30px;color:var(--dim);font-size:.8rem;line-height:1.6}
  /* solver */
  .field{display:flex;align-items:center;gap:8px;margin:8px 0}
  .field .lab{flex:1;color:var(--dim);font-size:.85rem}
  .field input,.field select{width:120px;padding:10px;background:var(--bg);color:var(--ink);
    border:1px solid var(--line);border-radius:8px;font-family:var(--mono);font-size:1rem}
  .field input:disabled{color:var(--good);opacity:.9}
  .lm{width:64px;padding:8px;border-radius:8px;border:1px solid var(--line);font-weight:700;
    font-size:.8rem;background:var(--panel);color:var(--dim)}
  .lm.live{background:var(--good);color:#00140a;border-color:var(--good)}
  .solveBtn{width:100%;padding:16px;margin-top:12px;background:var(--fire);color:#1a1200;
    border:none;border-radius:12px;font-weight:800;font-size:1.1rem;letter-spacing:.05em}
  .res{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}
  .res .card{text-align:center}
  .res .big{color:var(--fire)}
  .warn{color:var(--warn);font-size:.78rem;margin-top:10px;line-height:1.5}
  .hide{display:none}
  /* Basic / Forecast section switch */
  .sections{display:flex;gap:8px;margin:10px 0 2px}
  .sec{flex:1;padding:14px;border:1px solid var(--line);background:var(--panel);color:var(--dim);
    border-radius:12px;font-weight:800;letter-spacing:.1em;font-size:.95rem;text-align:center}
  .sec.on{background:var(--ink);color:var(--bg);border-color:var(--ink)}
  /* stability */
  .stabBadge{border-radius:16px;padding:20px;text-align:center;font-weight:800}
  .stabBadge .word{font-size:2.4rem;letter-spacing:.06em}
  .stabBadge .sub{font-size:.8rem;opacity:.85;margin-top:4px;font-family:var(--mono)}
  .bg-good{background:rgba(57,217,138,.16);border:1px solid var(--good);color:var(--good)}
  .bg-warnY{background:rgba(255,210,63,.15);border:1px solid var(--warnY);color:var(--warnY)}
  .bg-warn{background:rgba(255,107,107,.15);border:1px solid var(--warn);color:var(--warn)}
  .win{border-radius:10px;padding:10px;text-align:center;border:1px solid var(--line)}
  .win .wt{font-size:.7rem;color:var(--dim);letter-spacing:.06em}
  .win .wv{font-family:var(--mono);font-weight:700;font-size:1rem;margin-top:3px}
</style></head><body><div class="wrap">

<header><div class="brand">WIND LAB</div><div id="status" class="status off">connecting</div></header>
<div class="sections">
  <div id="secBasic" class="sec on" onclick="setSection('basic')">BASIC</div>
  <div id="secFore" class="sec" onclick="setSection('forecast')">FORECAST</div>
</div>
<div id="tabs" class="tabs"></div>

<!-- STABILITY (Forecast / Prediction tab only) -->
<div id="stabView" class="hide">
  <h2 id="stabTitle">Stability</h2>
  <div id="sentBadge" class="stabBadge bg-warn hide" style="margin-bottom:10px"><div class="word">--</div><div class="sub"></div></div>
  <div id="windHold" class="win" style="margin-bottom:10px"><div class="wt">WIND HOLD</div><div class="wv">--</div></div>
  <div id="stabBadge" class="stabBadge bg-warn"><div class="word">--</div><div class="sub"></div></div>
  <div class="grid" style="grid-template-columns:1fr 1fr 1fr 1fr;margin-top:10px" id="stabWindows"></div>
  <div id="stabThPanel" class="wheelWrap" style="margin-top:14px"></div>
  <div id="meterThPanel" class="wheelWrap" style="margin-top:14px"></div>
  <div id="sentPanel" class="wheelWrap" style="margin-top:14px"></div>
</div>

<!-- WIND VIEW (avg / node) -->
<div id="windView">
  <div class="hero">
    <div class="heroTop"><span class="heroLbl" id="viewName">Average</span>
      <span class="heroLbl" id="gustLbl"></span></div>
    <div><span class="spd" id="spd">--</span><span class="unit">MPH</span></div>
    <div style="margin-top:10px"><span class="rel" id="rel">--</span>
      <span class="unit">REL</span>
      <div class="relSub" id="relSub"></div></div>
    <div class="row">
      <div class="card"><div class="k">Speed SD</div><div class="v" id="spdSd">--</div></div>
      <div class="card"><div class="k">Dir SD</div><div class="v" id="dirSd">--</div></div>
    </div>
  </div>
  <h2>Telemetry</h2><div class="grid" id="tele"></div>
  <div id="weightsPanel" class="wheelWrap hide" style="margin-top:14px"></div>
  <div id="metersPanel" class="wheelWrap hide" style="margin-top:14px"></div>
</div>

<!-- SOLVER VIEW -->
<div id="solveView" class="hide">
  <h2>Rifle profile</h2>
  <div class="wheelWrap">
    <select id="profileSel" onchange="loadProfile(this.value)" style="width:100%;padding:12px;
      background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:10px;
      font-family:var(--mono);font-size:1rem"></select>
    <div class="stepRow">
      <button class="step" onclick="newProfile()">New</button>
      <button class="step" onclick="saveActive()">Save</button>
      <button class="step" onclick="saveAsProfile()">Save As</button>
    </div>
    <div class="stepRow">
      <button class="step" onclick="renameProfile()">Rename</button>
      <button class="step" onclick="deleteProfile()">Delete</button>
    </div>
    <div class="stepRow">
      <button class="step" onclick="exportProfiles()">Export .json</button>
      <button class="step" onclick="document.getElementById('importFile').click()">Import .json</button>
    </div>
    <input type="file" id="importFile" accept="application/json,.json" style="display:none" onchange="importProfiles(this)">
  </div>
  <h2>Ballistic solver</h2>
  <div class="wheelWrap">
    <div id="loadFields"></div>
    <button class="solveBtn" onclick="doSolve()">Solve</button>
    <div class="res" id="results"></div>
    <div class="warn">Unvalidated. Verify against a known solver before trusting for real dope. Drag tables are approximate standard curves.</div>
  </div>
</div>

<!-- WIND WHEEL (shared, always present) -->
<h2>Wind wheel &mdash; what the bullet sees</h2>
<div class="wheelWrap">
  <svg viewBox="0 0 200 200">
    <circle class="compass-ring" cx="100" cy="100" r="92"></circle>
    <g id="ticks"></g>
    <text class="card-n" x="100" y="20" text-anchor="middle">N</text>
    <text class="card-n" x="184" y="105" text-anchor="middle">E</text>
    <text class="card-n" x="100" y="192" text-anchor="middle">S</text>
    <text class="card-n" x="16" y="105" text-anchor="middle">W</text>
    <g id="fireArrow"><line x1="100" y1="100" x2="100" y2="18" stroke="var(--fire)" stroke-width="5" stroke-linecap="round"></line>
      <polygon points="100,10 94,24 106,24" fill="var(--fire)"></polygon></g>
    <g id="windArrow"><line x1="100" y1="100" x2="100" y2="30" stroke="var(--wind)" stroke-width="4" stroke-linecap="round"></line>
      <polygon points="100,22 95,34 105,34" fill="var(--wind)"></polygon></g>
    <circle cx="100" cy="100" r="6" fill="var(--ink)"></circle>
  </svg>
  <div class="legend"><span><span class="dot" style="background:var(--fire)"></span>Line of fire</span>
    <span><span class="dot" style="background:var(--wind)"></span>Wind flow</span></div>
  <div class="readouts">
    <div class="card"><div class="k">Clock</div><div class="big" id="clock">--</div></div>
    <div class="card"><div class="k">Rel. angle</div><div class="big" id="relW">--</div></div>
    <div class="card"><div class="k">Crosswind</div><div class="big" id="cross">--</div></div>
    <div class="card"><div class="k">Head / Tail</div><div class="big" id="ht">--</div></div>
  </div>
  <div class="ctrl">
    <label><span>Firing azimuth (toward target)</span><span class="azVal" id="azVal">0&deg;</span></label>
    <input type="range" id="az" min="0" max="359" value="0">
    <div class="stepRow"><button class="step" onclick="bump(-5)">&minus;5</button>
      <button class="step" onclick="bump(-1)">&minus;1</button>
      <button class="step" onclick="bump(1)">+1</button>
      <button class="step" onclick="bump(5)">+5</button></div>
  </div>
</div>

<footer id="base">Base station: waiting&hellip;</footer>
</div>
<script>
var data=null, view='avg';
var section=localStorage.getItem('section')||'basic';
if(section==='forecast')view='pred';
var fireAz=parseInt(localStorage.getItem('fireAz')||'0',10)||0;

// ---- load profile (all manual) + live-capable environment inputs ----
var LOAD=[
 {k:'mv',l:'Muzzle vel (fps)',d:2700},{k:'bc',l:'BC',d:0.243},
 {k:'model',l:'Drag model',d:7,opt:[[7,'G7'],[1,'G1']]},
 {k:'wgt',l:'Weight (gr)',d:175},{k:'cal',l:'Caliber (in)',d:0.308},
 {k:'twist',l:'Twist (in/turn)',d:10},{k:'blen',l:'Bullet length (in)',d:1.24},
 {k:'twistDir',l:'Twist dir',d:1,opt:[[1,'Right'],[-1,'Left']]},
 {k:'sh',l:'Sight height (in)',d:1.75},{k:'zero',l:'Zero (yd)',d:100},
 {k:'range',l:'Target range (yd)',d:500},
 {k:'zeroElevMag',l:'Group off. elev (MOA)',d:0},
 {k:'zeroElevDir',l:'Group landed (elev)',d:'high',opt:[['high','High'],['low','Low']]},
 {k:'zeroWindMag',l:'Group off. wind (MOA)',d:0},
 {k:'zeroWindDir',l:'Group landed (wind)',d:'right',opt:[['right','Right'],['left','Left']]}
];
// live-capable: temp, pressure, wind speed, wind angle
var LIVE=[
 {k:'tempF',l:'Temperature (F)',d:59,live:'live_temp'},
 {k:'pres',l:'Pressure (inHg)',d:29.92,live:'live_pres'},
 {k:'windmph',l:'Wind speed (mph)',d:10,live:'live_wind'},
 {k:'windrel',l:'Wind angle (rel)',d:90,live:'live_wind'},
 {k:'lat',l:'Latitude (deg)',d:45,live:'live_lat'}
];
function getv(k,d){var v=localStorage.getItem('sol_'+k);return v===null?d:v;}
function setv(k,v){localStorage.setItem('sol_'+k,v);}
function liveOn(key){return (localStorage.getItem(key)||'1')==='1';}
function toggleLive(key){localStorage.setItem(key,liveOn(key)?'0':'1');renderSolver();}

// ---- rifle profiles (saved on this device; export/import as .json) ----
// A profile captures the rifle/load block + zero offsets + spin/jump toggles.
// NOT captured: target range, temp/pressure/wind, Live/Manual toggles.
function profileFields(){return LOAD.map(function(f){return f.k;}).filter(function(k){return k!=='range';});}
function loadProfiles(){try{return JSON.parse(localStorage.getItem('profiles')||'{}');}catch(e){return {};}}
function storeProfiles(o){localStorage.setItem('profiles',JSON.stringify(o));}
function activeName(){return localStorage.getItem('activeProfile')||'';}
function setActive(n){localStorage.setItem('activeProfile',n);}

function snapshot(){var p={};profileFields().forEach(function(k){p[k]=getv(k,'');});
  p.__spin=localStorage.getItem('opt_spin')||'1';p.__jump=localStorage.getItem('opt_jump')||'1';return p;}
function applyProfile(p){
  profileFields().forEach(function(k){if(p[k]!==undefined)setv(k,p[k]);});
  if(p.__spin!==undefined)localStorage.setItem('opt_spin',p.__spin);
  if(p.__jump!==undefined)localStorage.setItem('opt_jump',p.__jump);
}
function saveProfile(name){if(!name)return;var all=loadProfiles();all[name]=snapshot();storeProfiles(all);setActive(name);renderSolver();}
function saveActive(){var n=activeName();if(!n){saveAsProfile();return;}saveProfile(n);flash('Saved "'+n+'"');}
function saveAsProfile(){var n=prompt('Profile name:',activeName()||'My rifle');if(n)saveProfile(n.trim());}
function renameProfile(){var n=activeName();if(!n)return;var nn=prompt('Rename profile:',n);if(!nn||nn===n)return;
  var all=loadProfiles();all[nn.trim()]=all[n];delete all[n];storeProfiles(all);setActive(nn.trim());renderSolver();}
function deleteProfile(){var n=activeName();if(!n)return;if(!confirm('Delete profile "'+n+'"?'))return;
  var all=loadProfiles();delete all[n];storeProfiles(all);setActive('');renderSolver();}
function loadProfile(name){if(!name)return;var all=loadProfiles();if(!all[name])return;applyProfile(all[name]);setActive(name);renderSolver();doSolve();}
function newProfile(){
  if(!confirm('Start a new rifle profile?\nCurrent fields will reset to defaults. Saved profiles are not affected.'))return;
  profileFields().forEach(function(k){localStorage.removeItem('sol_'+k);});   // getv() falls back to LOAD defaults
  localStorage.setItem('opt_spin','1');localStorage.setItem('opt_jump','1');
  setActive('');                                                             // dropdown -> "-- select / unsaved --"
  renderSolver();
  flash('New profile - enter rifle data, then Save As');
}

function exportProfiles(){
  var blob=new Blob([JSON.stringify(loadProfiles(),null,2)],{type:'application/json'});
  var url=URL.createObjectURL(blob);var a=document.createElement('a');
  a.href=url;a.download='rifle_profiles.json';document.body.appendChild(a);a.click();
  document.body.removeChild(a);URL.revokeObjectURL(url);
}
function importProfiles(input){
  var f=input.files[0];if(!f)return;var r=new FileReader();
  r.onload=function(){try{var imp=JSON.parse(r.result);var all=loadProfiles();var c=0;
    Object.keys(imp).forEach(function(k){all[k]=imp[k];c++;});storeProfiles(all);renderSolver();
    flash('Imported '+c+' profile(s)');
  }catch(e){alert('Import failed: not a valid profiles file.');}};
  r.readAsText(f);input.value='';
}
function flash(msg){var s=document.getElementById('status');var old=s.textContent;
  s.textContent=msg;setTimeout(function(){s.textContent=old;},1500);}

function card(deg){var n=["N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"];
  return n[Math.round(((deg%360)+360)%360/22.5)%16];}
function relAngle(fromDeg){return (((fromDeg-fireAz)%360)+360)%360;}

(function(){var t="";for(var a=0;a<360;a+=30){var r1=(a%90===0)?78:83,r2=90,rad=(a-90)*Math.PI/180;
  t+='<line class="tick" x1="'+(100+r1*Math.cos(rad))+'" y1="'+(100+r1*Math.sin(rad))+
     '" x2="'+(100+r2*Math.cos(rad))+'" y2="'+(100+r2*Math.sin(rad))+'"/>';}
  document.getElementById('ticks').innerHTML=t;})();

var azEl=document.getElementById('az');azEl.value=fireAz;
azEl.addEventListener('input',function(){fireAz=parseInt(azEl.value,10);saveAz();drawWheel();if(view==='solver')maybeAutoSolve();});
function bump(d){fireAz=((fireAz+d)%360+360)%360;azEl.value=fireAz;saveAz();drawWheel();if(view==='solver')maybeAutoSolve();}
function saveAz(){localStorage.setItem('fireAz',fireAz);document.getElementById('azVal').innerHTML=fireAz+'&deg;';}

// ---- per-meter weighting (client-side; overrides the firmware's plain average) ----
// Each active node can be given a weight (in %) reflecting how well that meter's
// wind represents the actual flight path (e.g. a node near the target vs one at
// the muzzle). Weights are auto-normalized at compute time, so they don't need
// to add up to exactly 100 - a node with no stored weight gets an equal share.
// Stored per node id in localStorage, separate from rifle profiles (this
// describes meter placement for the range session, not the rifle).
function nodeWeights(){try{return JSON.parse(localStorage.getItem('nodeWeights')||'{}');}catch(e){return {};}}
function storeWeights(w){localStorage.setItem('nodeWeights',JSON.stringify(w));}
// per-node speed multiplier (e.g. a meter at max ordinate sped up for winds aloft)
function nodeMults(){try{return JSON.parse(localStorage.getItem('nodeMult')||'{}');}catch(e){return {};}}
function storeMults(m){localStorage.setItem('nodeMult',JSON.stringify(m));}
function getMult(id){var m=nodeMults();var v=(m[id]!==undefined)?parseFloat(m[id]):1;return (isFinite(v)&&v>0)?v:1;}
function setNodeMult(id,v){var m=nodeMults();var f=parseFloat(v);m[id]=(isFinite(f)&&f>0)?f:1;storeMults(m);render();}
function resetMultsOne(){var ns=(data&&data.nodes)||[];var m={};ns.forEach(function(n){m[n.id]=1;});storeMults(m);renderWeightsPanel(true);render();}
function weightedAvgWind(){
  var ns=(data&&data.nodes)||[];
  if(!ns.length)return{speed:0,dir:0,speedSd:0,dirSd:0,count:0};
  var w=nodeWeights(),total=0;
  ns.forEach(function(n){total+=(w[n.id]!==undefined)?Math.max(0,w[n.id]):(100/ns.length);});
  if(total<=0)total=ns.length;
  var sumSpd=0,sumSpdSd=0,sumDirSd=0,sx=0,sy=0;
  ns.forEach(function(n){
    var wt=(w[n.id]!==undefined)?Math.max(0,w[n.id]):(100/ns.length);
    var f=wt/total, ml=getMult(n.id);
    sumSpd+=f*n.speed*ml;sumSpdSd+=f*n.speedSd*ml;sumDirSd+=f*n.dirSd;
    var r=n.dir*Math.PI/180;sx+=f*Math.cos(r);sy+=f*Math.sin(r);
  });
  var dir=Math.atan2(sy,sx)*180/Math.PI;if(dir<0)dir+=360;
  return{speed:sumSpd,dir:dir,speedSd:sumSpdSd,dirSd:sumDirSd,count:ns.length};
}
function setNodeWeight(id,v){var w=nodeWeights();w[id]=Math.max(0,parseFloat(v)||0);storeWeights(w);updateWeightTotal();render();}
function resetWeightsEqual(){
  var ns=(data&&data.nodes)||[];if(!ns.length)return;
  var each=Math.round((100/ns.length)*10)/10,w={};
  ns.forEach(function(n){w[n.id]=each;});
  storeWeights(w);renderWeightsPanel(true);render();
}
// ---- Phase 2: sensitivity-zone auto weighting (positions -> weights) --------
// Weights come from each meter's spot along the bullet's flight path: wind near
// the muzzle matters far more than wind near the target (~6x). Position source
// is GPS by default (base GPS + firing azimuth + node GPS, all on the box); a
// per-node manual downrange (yd) overrides GPS for that node. The heavy solve
// runs on the receiver and is cached there -- this only fires on the button.
var lastSens={};   // id -> {downYd,src,wPct} from the last /sensitivity call
function nodeDownManual(){try{return JSON.parse(localStorage.getItem('nodeDownManual')||'{}');}catch(e){return {};}}
function storeDownManual(m){localStorage.setItem('nodeDownManual',JSON.stringify(m));}
function setNodeDown(id,v){var m=nodeDownManual();
  if(v===''||v===null||isNaN(parseFloat(v)))delete m[id];else m[id]=parseFloat(v);
  storeDownManual(m);}
function clearManualDown(){localStorage.removeItem('nodeDownManual');flash('manual downrange cleared');renderWeightsPanel(true);}
function sensReadout(id){
  var r=lastSens[id];if(!r)return 'downrange: press Auto to compute';
  if(r.src==='none')return 'no GPS fix and no manual downrange';
  var dy=(r.downYd<0)?'?':Math.round(r.downYd)+' yd';
  var tag=(r.wPct<=0.05)?'off the flight path (0%)':(Math.round(r.wPct)+'% weight');
  return dy+' \u00b7 '+r.src+' \u00b7 '+tag;
}
async function autoWeights(){
  if(!data){flash('no data yet');return;}
  if(!activeName()){flash('pick a rifle profile first');return;}
  var p=solverParams();
  var q='/sensitivity?mv='+p.mv+'&bc='+p.bc+'&model='+p.model+'&wgt='+p.wgt+'&cal='+p.cal+
    '&twist='+p.twist+'&blen='+p.blen+'&sh='+p.sh+'&zero='+p.zero+'&range='+p.range+
    '&tempF='+p.tempF+'&pres='+p.pres+'&az='+fireAz;
  var man=nodeDownManual();
  Object.keys(man).forEach(function(id){q+='&dr'+id+'='+man[id];});
  try{
    var r=await fetch(q,{cache:'no-store'});var s=await r.json();
    if(!s.ok){flash('sensitivity: solver could not converge');return;}
    lastSens={};var w=nodeWeights(),none=0;
    (s.nodes||[]).forEach(function(nd){
      lastSens[nd.id]={downYd:nd.downYd,src:nd.src,wPct:nd.wPct};
      if(nd.src==='none')none++;
      w[nd.id]=Math.round(nd.wPct*10)/10;
    });
    storeWeights(w);renderWeightsPanel(true);render();
    flash('weights set from ballistics'+(none?(' - '+none+' node(s) need a position'):''));
  }catch(e){flash('sensitivity request failed');}
}
var weightsBuiltFor='';
function renderWeightsPanel(force){
  var panel=document.getElementById('weightsPanel');if(!panel)return;
  var ns=(data&&data.nodes)||[];
  if((view!=='avg'&&view!=='pred')||ns.length<2){panel.className='wheelWrap hide';return;}
  panel.className='wheelWrap';
  var sig=ns.map(function(n){return n.id;}).sort(function(a,b){return a-b;}).join(',');
  if(force||sig!==weightsBuiltFor){
    weightsBuiltFor=sig;
    var w=nodeWeights(),m=nodeMults(),dman=nodeDownManual();
    var istyle="width:100%;margin-top:4px;padding:8px;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.95rem";
    var h='<h2 style="margin:0 0 10px">Meter weights &amp; multipliers</h2>'+
      '<div style="color:var(--dim);font-size:.78rem;margin-bottom:10px">'+
      '<b>Auto from ballistics</b> sets each weight from the meter\'s spot on the flight path '+
      '(wind near the muzzle counts ~6&times; more than near the target). Position is GPS by '+
      'default; type a <b>Downrange (yd)</b> to override that meter. '+
      '<b>Multiplier &times;</b> scales this meter\'s wind speed (e.g. a meter at max ordinate for winds aloft).</div>';
    ns.forEach(function(n){
      var v=(w[n.id]!==undefined)?w[n.id]:Math.round((100/ns.length)*10)/10;
      var mv=(m[n.id]!==undefined)?m[n.id]:1;
      var dv=(dman[n.id]!==undefined)?dman[n.id]:'';
      h+='<div style="margin:10px 0;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
         '<div style="color:var(--dim);font-size:.85rem;margin-bottom:8px">Node '+n.id+' <span id="wlbl_'+n.id+'">('+n.speed.toFixed(1)+' mph)</span></div>'+
         '<div style="display:flex;gap:10px">'+
         '<label style="flex:1;color:var(--dim);font-size:.72rem">Weight %<input type="number" step="any" min="0" style="'+istyle+'" value="'+v+'" onchange="setNodeWeight('+n.id+',this.value)"></label>'+
         '<label style="flex:1;color:var(--dim);font-size:.72rem">Multiplier &times;<input type="number" step="0.05" min="0" style="'+istyle+'" value="'+mv+'" onchange="setNodeMult('+n.id+',this.value)"></label>'+
         '</div>'+
         '<div style="display:flex;gap:10px;margin-top:8px"><label style="flex:1;color:var(--dim);font-size:.72rem">Downrange (yd) <span style="opacity:.7">blank = GPS</span>'+
         '<input type="number" step="any" min="0" style="'+istyle+'" value="'+dv+'" onchange="setNodeDown('+n.id+',this.value)"></label></div>'+
         '<div id="slbl_'+n.id+'" style="color:var(--dim);font-size:.72rem;margin-top:6px">'+sensReadout(n.id)+'</div>'+
         '</div>';
    });
    h+='<div class="stepRow"><button class="step" onclick="autoWeights()">Auto from ballistics</button>'+
       '<button class="step" onclick="resetWeightsEqual()">Equal split</button></div>'+
       '<div class="stepRow"><button class="step" onclick="clearManualDown()">Clear manual downrange</button>'+
       '<button class="step" onclick="resetMultsOne()">Reset &times;1</button></div>'+
       '<div id="wTotal" style="color:var(--dim);font-size:.78rem;margin-top:8px;text-align:center"></div>';
    panel.innerHTML=h;
  }else{
    ns.forEach(function(n){var e=document.getElementById('wlbl_'+n.id);if(e)e.textContent='('+n.speed.toFixed(1)+' mph)';
      var se=document.getElementById('slbl_'+n.id);if(se)se.textContent=sensReadout(n.id);});
  }
  updateWeightTotal();
}
function updateWeightTotal(){
  var e=document.getElementById('wTotal');if(!e||!data)return;
  var ns=data.nodes||[],w=nodeWeights(),total=0;
  ns.forEach(function(n){total+=(w[n.id]!==undefined)?Math.max(0,w[n.id]):(100/ns.length);});
  e.textContent='Total '+total.toFixed(0)+'%'+(Math.abs(total-100)>0.5?' (auto-normalized)':' \u2713');
}

var metersBuiltFor='';
function renderMetersPanel(force){
  var panel=document.getElementById('metersPanel');if(!panel)return;
  var ns=(data&&data.nodes)||[];
  if(view!=='avg'&&view!=='pred'||!ns.length){panel.className='wheelWrap hide';return;}
  panel.className='wheelWrap';
  var sig=ns.map(function(n){return n.id+':'+getNodeMeterType(n.id);}).join(',');
  if(!force&&sig===metersBuiltFor)return;
  metersBuiltFor=sig;
  var istyle="width:100%;margin-top:4px;padding:6px;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.85rem";
  var h='<h2 style="margin:0 0 10px">Wind meters</h2>'+
    '<div style="color:var(--dim);font-size:.78rem;margin-bottom:10px">'+
    'Pick each node\'s meter type. "Meter quality" on its tab is judged against '+
    'that type\'s own SD thresholds below, not a one-size-fits-all bar.</div>';
  ns.forEach(function(n){
    var cur=getNodeMeterType(n.id);
    h+='<div style="margin:8px 0;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
       '<div style="color:var(--dim);font-size:.85rem;margin-bottom:6px">Node '+n.id+'</div>'+
       '<select style="'+istyle+'" onchange="setNodeMeterType('+n.id+',this.value)">'+
       '<option value="-1"'+(cur===-1?' selected':'')+'>-- not set --</option>'+
       '<option value="0"'+(cur===0?' selected':'')+'>SparkFun</option>'+
       '<option value="1"'+(cur===1?' selected':'')+'>Inspeed</option>'+
       '<option value="2"'+(cur===2?' selected':'')+'>Calypso</option>'+
       '</select></div>';
  });
  panel.innerHTML=h;
}

// ---- stability index (Forecast) ----
// Both speed SD and direction SD must clear a band for that tier; worse governs.
// stabTh is the AGGREGATE badge threshold (GO/CAUTION/WAIT). It is separate from
// meterTh which is per-node Meter Quality only.
var STAB_TH_DEFAULT={gDir:10,gSpd:3,yDir:20,ySpd:6};
function stabTh(){try{var t=JSON.parse(localStorage.getItem('stabTh'));return t||{};}catch(e){return {};}}
function storeStabTh(t){localStorage.setItem('stabTh',JSON.stringify(t));}
function TH(k){var t=stabTh();return (t[k]!==undefined)?parseFloat(t[k]):STAB_TH_DEFAULT[k];}
function setStabTh(k,v){var t=stabTh();t[k]=parseFloat(v)||0;storeStabTh(t);renderStabThPanel(true);render();}
function resetStabTh(){localStorage.removeItem('stabTh');renderStabThPanel(true);render();}
function tier(spdSd,dirSd){
  if(dirSd<=TH('gDir') && spdSd<=TH('gSpd')) return {c:'good', lbl:'STABLE',   go:'GO'};
  if(dirSd<=TH('yDir') && spdSd<=TH('ySpd')) return {c:'warnY',lbl:'CAUTION',  go:'CAUTION'};
  return {c:'warn', lbl:'UNSTABLE', go:'WAIT'};
}
var stabThBuilt=false;
function renderStabThPanel(force){
  var el=document.getElementById('stabThPanel');if(!el)return;
  if(!force&&stabThBuilt)return;      // don't rebuild on every poll -- kills mobile keyboard focus
  stabThBuilt=true;
  var t=stabTh(), isCustom=Object.keys(t).length>0;
  var is="width:100%;margin-top:4px;padding:8px;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.9rem";
  el.innerHTML='<h2 style="margin:0 0 8px">GO / CAUTION / WAIT thresholds</h2>'+
    '<div style="color:var(--dim);font-size:.78rem;margin-bottom:10px">'+
    'Applies to the big badge only. Both speed SD and dir SD must be under the green values '+
    'to show GO; both under yellow for CAUTION; anything worse = WAIT.</div>'+
    '<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">'+
    '<label style="color:var(--dim);font-size:.75rem">GO: dir SD max (&deg;)<input type="number" step="any" style="'+is+'" value="'+TH('gDir')+'" onchange="setStabTh(\'gDir\',this.value)"></label>'+
    '<label style="color:var(--dim);font-size:.75rem">GO: spd SD max (mph)<input type="number" step="any" style="'+is+'" value="'+TH('gSpd')+'" onchange="setStabTh(\'gSpd\',this.value)"></label>'+
    '<label style="color:var(--dim);font-size:.75rem">CAUTION: dir SD max (&deg;)<input type="number" step="any" style="'+is+'" value="'+TH('yDir')+'" onchange="setStabTh(\'yDir\',this.value)"></label>'+
    '<label style="color:var(--dim);font-size:.75rem">CAUTION: spd SD max (mph)<input type="number" step="any" style="'+is+'" value="'+TH('ySpd')+'" onchange="setStabTh(\'ySpd\',this.value)"></label>'+
    '</div>'+
    (isCustom?'<div style="margin-top:10px"><button class="step" onclick="resetStabTh()">Reset to defaults</button></div>':'')+
    '</div>';
}
// ---- per-node wind meter type (client-side; the node no longer reports this --
// see project notes on why the ATtiny/node cross-sync was dropped) ----
// Stored per node id in localStorage, same pattern as weights/multipliers.
var METER_NAMES = {0:'SparkFun', 1:'Inspeed', 2:'Calypso'};
function nodeMeterTypes(){try{return JSON.parse(localStorage.getItem('nodeMeterType')||'{}');}catch(e){return {};}}
function storeMeterTypes(m){localStorage.setItem('nodeMeterType',JSON.stringify(m));}
function getNodeMeterType(id){var m=nodeMeterTypes();return (m[id]!==undefined)?m[id]:-1;}  // -1 = not set yet
function setNodeMeterType(id,v){var m=nodeMeterTypes();m[id]=parseInt(v,10);storeMeterTypes(m);renderMetersPanel(true);render();}

// ---- per-meter-type SD threshold profiles (client-side, editable) ----
// Presets below reflect real, structural differences between the three sensor
// types -- not a guess at "which is better":
//  - SparkFun: an 8-position (45-degree step) resistive vane has a real
//    quantization noise floor of ~13 deg SD from discretization alone (45/sqrt(12)).
//    Thresholds are loosened so that floor isn't flagged as "unstable wind."
//  - Inspeed: continuous potentiometer/Hall vane, no discretization step. Uses
//    the same baseline as the original aggregate Stability Index defaults.
//  - Calypso: UNVERIFIED placeholder, same as Inspeed's baseline. An ultrasonic
//    sensor has no moving-part inertia, so it may legitimately show MORE
//    variation than a mechanical vane on real gusts (resolving fluctuations a
//    cup/vane's mechanical inertia would smooth out) -- that would need looser,
//    not tighter, thresholds. Don't trust this until characterized against real
//    hardware; update it here once you have field data.
var METER_TH_DEFAULT = {
  0: {gDir:20, gSpd:3, yDir:35, ySpd:6},   // SparkFun
  1: {gDir:10, gSpd:3, yDir:20, ySpd:6},   // Inspeed
  2: {gDir:10, gSpd:3, yDir:20, ySpd:6}    // Calypso -- UNVERIFIED, see note above
};
function meterTh(){try{return JSON.parse(localStorage.getItem('meterTh')||'{}');}catch(e){return {};}}
function storeMeterTh(t){localStorage.setItem('meterTh',JSON.stringify(t));}
function getMeterThresholds(type){var t=meterTh()[type];return t?t:METER_TH_DEFAULT[type]||METER_TH_DEFAULT[1];}
function setMeterThreshold(type,field,v){var t=meterTh();var cur=t[type]||Object.assign({},METER_TH_DEFAULT[type]||METER_TH_DEFAULT[1]);
  cur[field]=parseFloat(v)||0;t[type]=cur;storeMeterTh(t);renderMeterThPanel(true);render();}
function resetMeterThreshold(type){var t=meterTh();delete t[type];storeMeterTh(t);renderMeterThPanel(true);render();}
function meterTier(meterType,spdSd,dirSd){
  var t=getMeterThresholds(meterType);
  if(dirSd<=t.gDir && spdSd<=t.gSpd) return {c:'good', lbl:'GOOD'};
  if(dirSd<=t.yDir && spdSd<=t.ySpd) return {c:'warnY',lbl:'NOISY'};
  return {c:'warn', lbl:'V.NOISY'};
}
async function updateWindHold(){
  var el=document.getElementById('windHold');
  if(!el)return;
  if(!data){el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">--</div>';return;}
  if(!activeName()){el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">-- (no rifle profile)</div>';return;}
  try{
    var p=solverParams();
    var q='/solve?mv='+p.mv+'&bc='+p.bc+'&model='+p.model+'&wgt='+p.wgt+'&cal='+p.cal+
      '&twist='+p.twist+'&blen='+p.blen+'&twistDir='+p.twistDir+'&sh='+p.sh+'&zero='+p.zero+
      '&range='+p.range+'&tempF='+p.tempF+'&pres='+p.pres+'&windmph='+p.windmph+'&windrel='+p.windrel+
      '&lat='+p.lat+'&az='+fireAz+'&earth='+p.earth+'&spin='+p.spin+'&jump='+p.jump;
    var r=await fetch(q,{cache:'no-store'});var s=await r.json();
    if(!s.ok){el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">solver error</div>';return;}
    var wmag=parseFloat(getv('zeroWindMag',0))||0, wdir=getv('zeroWindDir','right');
    var zw=(wdir==='right')?-wmag:wmag;
    var wMOA=s.windMOA+zw, wMil=s.windMil+zw/3.4377;
    el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">'+Math.abs(wMOA).toFixed(1)+' '+(wMOA>=0?'R':'L')+
      ' MOA &middot; '+Math.abs(wMil).toFixed(2)+' '+(wMil>=0?'R':'L')+' mil</div>';
  }catch(e){el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">--</div>';}
}
function renderStability(){
  if(!data||!data.stability)return;
  var s=data.stability, cur=tier(s.cur.spdSd,s.cur.dirSd);
  var badge=document.getElementById('stabBadge');
  badge.className='stabBadge bg-'+cur.c;
  badge.querySelector('.word').textContent=cur.go;
  badge.querySelector('.sub').innerHTML='wind '+cur.lbl+' &middot; spd SD '+s.cur.spdSd.toFixed(1)+
    ' mph &middot; dir SD '+Math.round(s.cur.dirSd)+'&deg;';
  var wins=[['Now',s.cur],['10 s',s.w10],['30 s',s.w30],['60 s',s.w60]], h='';
  wins.forEach(function(p){var t=tier(p[1].spdSd,p[1].dirSd);
    h+='<div class="win bg-'+t.c+'"><div class="wt">'+p[0]+'</div>'+
       '<div class="wv">'+p[1].spdSd.toFixed(1)+' / '+Math.round(p[1].dirSd)+'&deg;</div></div>';});
  document.getElementById('stabWindows').innerHTML=h;
  renderStabThPanel();
  renderMeterThPanel();
  renderSentBadge();
  renderSentPanel();
}

var meterThBuilt=false;
function renderMeterThPanel(force){
  var el=document.getElementById('meterThPanel');if(!el)return;
  if(!force&&meterThBuilt)return;     // don't rebuild on every poll -- kills mobile keyboard focus
  meterThBuilt=true;
  var is="width:100%;margin-top:4px;padding:8px;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.9rem";
  var h='<h2 style="margin:0 0 8px">Meter quality thresholds</h2>'+
    '<div style="color:var(--dim);font-size:.78rem;margin-bottom:10px">'+
    'GOOD / NOISY / V.NOISY on each node\'s tab. Separate from the GO badge above. '+
    'SparkFun\'s looser dir defaults account for its 8-position vane (~13&deg; quantization floor).</div>';
  [0,1,2].forEach(function(type){
    var t=getMeterThresholds(type),isCustom=(meterTh()[type]!==undefined);
    h+='<div style="margin:8px 0;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
       '<div style="color:var(--dim);font-size:.85rem;margin-bottom:8px">'+METER_NAMES[type]+
       (isCustom?' <button class="step" style="padding:2px 8px;font-size:.7rem" onclick="resetMeterThreshold('+type+')">reset</button>':'')+
       '</div><div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">'+
       '<label style="color:var(--dim);font-size:.75rem">GOOD: dir SD (&deg;)<input type="number" step="any" style="'+is+'" value="'+t.gDir+'" onchange="setMeterThreshold('+type+',\'gDir\',this.value)"></label>'+
       '<label style="color:var(--dim);font-size:.75rem">GOOD: spd SD (mph)<input type="number" step="any" style="'+is+'" value="'+t.gSpd+'" onchange="setMeterThreshold('+type+',\'gSpd\',this.value)"></label>'+
       '<label style="color:var(--dim);font-size:.75rem">NOISY: dir SD (&deg;)<input type="number" step="any" style="'+is+'" value="'+t.yDir+'" onchange="setMeterThreshold('+type+',\'yDir\',this.value)"></label>'+
       '<label style="color:var(--dim);font-size:.75rem">NOISY: spd SD (mph)<input type="number" step="any" style="'+is+'" value="'+t.ySpd+'" onchange="setMeterThreshold('+type+',\'ySpd\',this.value)"></label>'+
       '</div></div>';
  });
  el.innerHTML=h;
}

// ============================================================================
// SENTINEL AGREEMENT  (Forecast tab only)
// ============================================================================
// A Sentinel is a user-designated upwind meter paired with 1+ downwind nodes.
// Each pair "agrees" when the two meters are within a per-pair speed % and
// direction deg tolerance. The big banner then counts how many pairs agree
// out of the total live pairs, and applies user-set green/yellow thresholds
// against that count. Missing meters (dropped from the receiver's active list)
// are excluded from both numerator and denominator -- 4/4 becomes 3/3 rather
// than 3/4, so a dead node doesn't drag the banner red.
// All state client-side, localStorage. No firmware/receiver changes.
var SENTINEL_PAIR_DEFAULT = {spdPct:20, dirDeg:15};   // your chosen looser default
var SENTINEL_AGG_DEFAULT  = {g:0.75, y:0.50};         // >=75% green, >=50% yellow

function sentinelEnabled(){return localStorage.getItem('sentOn')==='1';}
function setSentinelEnabled(v){localStorage.setItem('sentOn',v?'1':'0');sentPanelBuilt=false;renderStability();}

// pairs[] = [{sentId, nodeId, spdPct, dirDeg}, ...]
function sentPairs(){try{return JSON.parse(localStorage.getItem('sentPairs')||'[]');}catch(e){return [];}}
function storeSentPairs(p){localStorage.setItem('sentPairs',JSON.stringify(p));}
function addSentPair(sentId,nodeId){
  var p=sentPairs();
  // reject self-pair, duplicate pair
  if(sentId==nodeId){flash('sentinel and node must differ');return;}
  if(p.some(function(x){return x.sentId==sentId&&x.nodeId==nodeId;})){flash('pair already exists');return;}
  p.push({sentId:parseInt(sentId,10),nodeId:parseInt(nodeId,10),
          spdPct:SENTINEL_PAIR_DEFAULT.spdPct,dirDeg:SENTINEL_PAIR_DEFAULT.dirDeg});
  storeSentPairs(p);sentPanelBuilt=false;renderStability();
}
function removeSentPair(i){var p=sentPairs();p.splice(i,1);storeSentPairs(p);sentPanelBuilt=false;renderStability();}
function setPairTh(i,field,v){var p=sentPairs();if(!p[i])return;
  p[i][field]=parseFloat(v)||0;storeSentPairs(p);renderStability();}

function sentAggTh(){try{var t=JSON.parse(localStorage.getItem('sentAggTh'));return t||SENTINEL_AGG_DEFAULT;}catch(e){return SENTINEL_AGG_DEFAULT;}}
function storeSentAggTh(t){localStorage.setItem('sentAggTh',JSON.stringify(t));}
function setSentAggTh(k,v){var t=sentAggTh();t[k]=parseFloat(v)||0;storeSentAggTh(t);renderStability();}
function resetSentAggTh(){localStorage.removeItem('sentAggTh');sentPanelBuilt=false;renderStability();}

// Circular direction difference in degrees, always 0..180.
function dirDiff(a,b){var d=Math.abs(a-b)%360;return d>180?360-d:d;}

// Evaluate all pairs against current data. Returns {live:[{...pair, agree, spdDelta, dirDelta}], missing:N}
function evalSentinel(){
  var pairs=sentPairs();
  var ns=(data&&data.nodes)||[];
  var byId={};ns.forEach(function(n){byId[n.id]=n;});
  var live=[], missing=0;
  pairs.forEach(function(p){
    var s=byId[p.sentId], n=byId[p.nodeId];
    if(!s||!n){missing++;return;}       // dropped from receiver -> excluded
    var sd=(s.speed>0.1)?Math.abs(s.speed-n.speed)/s.speed*100:Math.abs(s.speed-n.speed)*100;
    var dd=dirDiff(s.dir,n.dir);
    var agree=(sd<=p.spdPct)&&(dd<=p.dirDeg);
    live.push({sentId:p.sentId,nodeId:p.nodeId,spdPct:p.spdPct,dirDeg:p.dirDeg,
               spdDelta:sd,dirDelta:dd,agree:agree});
  });
  return {live:live, missing:missing};
}

function sentinelTier(agreeCount, totalLive){
  if(totalLive<=0) return {c:'warn', lbl:'NO PAIRS', go:'--', ratio:0};
  var ratio = agreeCount/totalLive, th=sentAggTh();
  var pct = Math.round(ratio*100);
  if(ratio>=th.g) return {c:'good', lbl:pct+'% AGREE', go:'GO', ratio:ratio};
  if(ratio>=th.y) return {c:'warnY',lbl:pct+'% AGREE', go:'CAUTION', ratio:ratio};
  return {c:'warn', lbl:pct+'% AGREE', go:'WAIT', ratio:ratio};
}

function renderSentBadge(){
  var el=document.getElementById('sentBadge');if(!el)return;
  if(!sentinelEnabled()){el.className='stabBadge bg-warn hide';return;}
  var ev=evalSentinel();
  if(ev.live.length===0 && ev.missing===0){
    el.className='stabBadge bg-warn';
    el.querySelector('.word').textContent='--';
    el.querySelector('.sub').innerHTML='no sentinel pairs set';
    return;
  }
  var agreeN=ev.live.filter(function(p){return p.agree;}).length;
  var t=sentinelTier(agreeN, ev.live.length);
  el.className='stabBadge bg-'+t.c;
  el.querySelector('.word').textContent=t.go;
  var sub='SENTINEL &middot; '+agreeN+'/'+ev.live.length+' pairs agree';
  if(ev.missing>0) sub+=' &middot; '+ev.missing+' dropped';
  el.querySelector('.sub').innerHTML=sub;
}

// The config panel: rebuild-once pattern like the other threshold panels so a
// mid-poll rebuild can't dismiss the mobile keyboard.
var sentPanelBuilt=false;
function renderSentPanel(force){
  var el=document.getElementById('sentPanel');if(!el)return;
  if(!force&&sentPanelBuilt){updateSentReadouts();return;}
  sentPanelBuilt=true;

  var is="width:100%;margin-top:4px;padding:8px;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.9rem";
  var on=sentinelEnabled();

  var h='<h2 style="margin:0 0 8px">Sentinel agreement</h2>'+
        '<div style="color:var(--dim);font-size:.78rem;margin-bottom:10px">'+
        'Pair an upwind Sentinel meter with one or more downwind nodes. When their '+
        'wind readings agree within the per-pair tolerances, that pair counts toward '+
        'the Sentinel Agreement banner above WIND HOLD. Both banners green = ideal shot moment.</div>'+
        '<div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:12px">'+
        '<span style="color:var(--dim);font-size:.85rem">Enable Sentinel</span>'+
        '<button class="step" onclick="setSentinelEnabled('+(on?'false':'true')+')">'+(on?'ON':'OFF')+'</button>'+
        '</div>';

  if(!on){el.innerHTML=h;return;}

  // ---- Aggregate thresholds ----
  var agg=sentAggTh();
  h+='<div style="margin:8px 0 12px;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
     '<div style="color:var(--dim);font-size:.85rem;margin-bottom:8px">Banner thresholds (fraction of live pairs that must agree)</div>'+
     '<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">'+
     '<label style="color:var(--dim);font-size:.75rem">GO: agree fraction (0-1)<input type="number" step="0.05" min="0" max="1" style="'+is+'" value="'+agg.g+'" onchange="setSentAggTh(\'g\',this.value)"></label>'+
     '<label style="color:var(--dim);font-size:.75rem">CAUTION: agree fraction (0-1)<input type="number" step="0.05" min="0" max="1" style="'+is+'" value="'+agg.y+'" onchange="setSentAggTh(\'y\',this.value)"></label>'+
     '</div>'+
     '<div style="margin-top:8px"><button class="step" onclick="resetSentAggTh()">Reset thresholds</button></div>'+
     '</div>';

  // ---- Existing pairs ----
  var pairs=sentPairs();
  h+='<h2 style="margin:14px 0 8px">Pairs</h2>';
  if(pairs.length===0){
    h+='<div style="color:var(--dim);font-size:.78rem;margin-bottom:10px">No pairs yet. Add one below.</div>';
  }else{
    pairs.forEach(function(p,i){
      h+='<div style="margin:8px 0;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
         '<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">'+
         '<div style="color:var(--ink);font-size:.9rem;font-weight:700">Sentinel '+p.sentId+' &rarr; Node '+p.nodeId+'</div>'+
         '<button class="step" style="padding:2px 8px;font-size:.75rem" onclick="removeSentPair('+i+')">remove</button>'+
         '</div>'+
         '<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">'+
         '<label style="color:var(--dim);font-size:.72rem">Speed tolerance (%)<input type="number" step="any" min="0" style="'+is+'" value="'+p.spdPct+'" onchange="setPairTh('+i+',\'spdPct\',this.value)"></label>'+
         '<label style="color:var(--dim);font-size:.72rem">Direction tolerance (&deg;)<input type="number" step="any" min="0" style="'+is+'" value="'+p.dirDeg+'" onchange="setPairTh('+i+',\'dirDeg\',this.value)"></label>'+
         '</div>'+
         '<div id="pair_readout_'+i+'" style="color:var(--dim);font-size:.72rem;margin-top:6px">--</div>'+
         '</div>';
    });
  }

  // ---- Add-pair widget ----
  var ns=(data&&data.nodes)||[];
  var opts='<option value="">select node</option>';
  ns.forEach(function(n){opts+='<option value="'+n.id+'">Node '+n.id+'</option>';});
  h+='<div style="margin:12px 0 8px;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
     '<div style="color:var(--dim);font-size:.85rem;margin-bottom:8px">Add pair</div>'+
     '<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px">'+
     '<label style="color:var(--dim);font-size:.72rem">Sentinel (upwind)<select id="newSentSel" style="'+is+'">'+opts+'</select></label>'+
     '<label style="color:var(--dim);font-size:.72rem">Paired node<select id="newNodeSel" style="'+is+'">'+opts+'</select></label>'+
     '</div>'+
     '<button class="step" onclick="addSentPair(document.getElementById(\'newSentSel\').value,document.getElementById(\'newNodeSel\').value)">Add pair</button>'+
     '</div>';

  el.innerHTML=h;
  updateSentReadouts();
}

// Live update the per-pair readout lines without touching the input DOM
// (same pattern as renderWeightsPanel's else-branch -- protects mobile keyboard).
function updateSentReadouts(){
  if(!sentinelEnabled())return;
  var ev=evalSentinel();
  var pairs=sentPairs();
  var iByKey={};
  pairs.forEach(function(p,i){iByKey[p.sentId+'>'+p.nodeId]=i;});
  ev.live.forEach(function(r){
    var i=iByKey[r.sentId+'>'+r.nodeId];if(i===undefined)return;
    var el=document.getElementById('pair_readout_'+i);if(!el)return;
    var color=r.agree?'var(--good)':'var(--warn)';
    el.innerHTML='<span style="color:'+color+'">'+(r.agree?'AGREE':'DISAGREE')+'</span>'+
      ' &middot; &Delta;speed '+r.spdDelta.toFixed(1)+'% &middot; &Delta;dir '+Math.round(r.dirDelta)+'&deg;';
  });
  // mark dropped pairs
  pairs.forEach(function(p,i){
    var live=ev.live.some(function(r){return r.sentId===p.sentId&&r.nodeId===p.nodeId;});
    if(!live){var el=document.getElementById('pair_readout_'+i);
      if(el)el.innerHTML='<span style="color:var(--dim)">meter offline &middot; excluded from count</span>';}
  });
}

function avgWind(){if(!data)return{speed:0,dir:0};var a=weightedAvgWind();return{speed:a.speed,dir:a.dir};}
function curWind(){
  if(!data)return{ok:false,speed:0,dir:0,speedSd:0,dirSd:0};
  if(view==='avg'||view==='pred'||view==='solver'){var a=weightedAvgWind();return{ok:a.count>0,speed:a.speed,dir:a.dir,speedSd:a.speedSd,dirSd:a.dirSd,gust:null};}
  var n=(data.nodes||[]).find(function(x){return x.id==view;});
  if(!n)return{ok:false,speed:0,dir:0,speedSd:0,dirSd:0};
  return{ok:true,speed:n.speed,dir:n.dir,speedSd:n.speedSd,dirSd:n.dirSd,gust:n.gust,node:n};
}

function drawWheel(){
  var w=curWind();var windFrom=w.dir||0,spd=w.speed||0;
  document.getElementById('fireArrow').setAttribute('transform','rotate('+fireAz+' 100 100)');
  document.getElementById('windArrow').setAttribute('transform','rotate('+((windFrom+180)%360)+' 100 100)');
  var rel=relAngle(windFrom);
  var cross=spd*Math.sin(rel*Math.PI/180), head=spd*Math.cos(rel*Math.PI/180);
  var hr=Math.round(rel/30)%12; if(hr===0)hr=12;
  document.getElementById('clock').textContent=hr+" o'clock";
  document.getElementById('relW').innerHTML=Math.round(rel)+'&deg;';
  document.getElementById('cross').textContent=Math.abs(cross).toFixed(1)+(cross>=0?' R':' L');
  document.getElementById('ht').textContent=Math.abs(head).toFixed(1)+(head>=0?' Head':' Tail');
}

function setSection(s){
  section=s;localStorage.setItem('section',s);
  document.getElementById('secBasic').className='sec'+(s==='basic'?' on':'');
  document.getElementById('secFore').className='sec'+(s==='forecast'?' on':'');
  view=(s==='forecast')?'pred':'avg';
  setView(view);
}
function tabs(){
  var t=document.getElementById('tabs');
  var first=(section==='forecast')?{v:'pred',l:'PRED'}:{v:'avg',l:'AVG'};
  var h='<div class="tab'+((view===first.v)?' on':'')+'" onclick="setView(\''+first.v+'\')">'+first.l+'</div>';
  (data.nodes||[]).forEach(function(n){h+='<div class="tab'+(view==n.id?' on':'')+'" onclick="setView('+n.id+')">'+n.id+'</div>';});
  h+='<div class="tab'+(view==='solver'?' on':'')+'" onclick="setView(\'solver\')">SOLVER</div>';
  t.innerHTML=h;
}
function setView(v){view=v;
  var isSolver=(v==='solver'), isPred=(v==='pred');
  document.getElementById('windView').className=isSolver?'hide':'';
  document.getElementById('solveView').className=isSolver?'':'hide';
  document.getElementById('stabView').className=isPred?'':'hide';
  if(isSolver)renderSolver();
  render();
}

function tele(){
  var g=document.getElementById('tele');
  function c(k,v){return '<div class="card"><div class="k">'+k+'</div><div class="v">'+v+'</div></div>';}
  if(view==='avg'||view==='pred'){var a=data.avg;var ww=weightedAvgWind();
    g.innerHTML=c('Nodes',a.count)+c('Wtd wind',ww.speed.toFixed(1)+' mph '+Math.round(ww.dir)+'&deg; '+card(ww.dir))+
      c('Avg temp',a.tempF.toFixed(0)+'&deg;F')+c('Avg baro',a.presInHg.toFixed(2)+' inHg');
  }else{var n=(data.nodes||[]).find(function(x){return x.id==view;});if(!n){g.innerHTML='';return;}
    var lux=(n.lux<0)?'&mdash;':n.lux.toFixed(0)+' lx',dist=(n.distYd<0)?'&mdash;':n.distYd+' yd';
    var ml=getMult(n.id);
    var mcard=(ml!==1)?c('Wind mult','&times;'+ml.toFixed(2)+' = '+(n.speed*ml).toFixed(1)+' mph'):'';
    var mt=getNodeMeterType(n.id);
    var vaneLbl=(mt>=0)?METER_NAMES[mt]:'not set';
    var mqcard=(mt>=0)?(function(){var mq=meterTier(mt,n.speedSd,n.dirSd);
      return '<div class="card"><div class="k">Meter quality</div><div class="v" style="color:var(--'+mq.c+')">'+mq.lbl+'</div></div>';})():'';
    g.innerHTML=c('True dir',Math.round(n.dir)+'&deg; '+card(n.dir))+c('Temp',n.tempF.toFixed(0)+'&deg;F')+
      c('Baro',n.presInHg.toFixed(2)+' inHg')+c('Light',lux)+
      c('Wind meter',vaneLbl)+mqcard+
      c('Battery',n.battV.toFixed(2)+'V &middot; '+n.battPct+'%')+c('Distance',dist)+
      c('Signal',n.rssi+' dBm')+c('Updated',n.age+'s ago')+mcard;
  }
}

function render(){
  if(!data)return;tabs();
  if(view==='solver'){drawWheel();syncLiveInputs();return;}
  var w=curWind();
  document.getElementById('viewName').textContent=(view==='avg'||view==='pred')?('Weighted avg / '+data.avg.count+' nodes'):'Node '+view;
  document.getElementById('spd').textContent=w.ok?w.speed.toFixed(1):'--';
  var rel=relAngle(w.dir);var hr=Math.round(rel/30)%12;if(hr===0)hr=12;
  document.getElementById('rel').innerHTML=w.ok?(Math.round(rel)+'&deg;'):'--';
  var dirLbl=(view==='avg'||view==='pred')?'weighted':'true';
  document.getElementById('relSub').innerHTML=w.ok?(hr+" o'clock &middot; "+dirLbl+" "+Math.round(w.dir)+'&deg; '+card(w.dir)):'';
  document.getElementById('spdSd').textContent=w.ok?('\u00B1'+w.speedSd.toFixed(1)):'--';
  document.getElementById('dirSd').innerHTML=w.ok?('\u00B1'+Math.round(w.dirSd)+'&deg;'):'--';
  document.getElementById('gustLbl').innerHTML=(w.gust!=null)?('Gust '+w.gust.toFixed(1)+' mph'):'';
  tele();drawWheel();renderWeightsPanel();renderMetersPanel();
  if(view==='pred'){renderStability();updateWindHold();}
  var b=data.base;
  document.getElementById('base').innerHTML='Base station &middot; '+
    (b.gps?(b.lat.toFixed(5)+', '+b.lon.toFixed(5)):'no GPS fix')+'<br>'+
    b.tempF.toFixed(0)+'&deg;F &middot; '+b.presInHg.toFixed(2)+' inHg &middot; battery '+b.battPct+'%';
}

// ---- solver UI ----
function renderSolver(){
  var h='';
  LOAD.forEach(function(f){
    if(f.opt){var o='';f.opt.forEach(function(p){o+='<option value="'+p[0]+'"'+((''+getv(f.k,f.d))==(''+p[0])?' selected':'')+'>'+p[1]+'</option>';});
      h+='<div class="field"><span class="lab">'+f.l+'</span><select onchange="setv(\''+f.k+'\',this.value)">'+o+'</select></div>';
    }else{
      h+='<div class="field"><span class="lab">'+f.l+'</span><input type="number" step="any" value="'+getv(f.k,f.d)+'" onchange="setv(\''+f.k+'\',this.value)"></div>';
    }
  });
  h+='<div class="field"><span class="lab" style="color:var(--good)">Spin drift</span><input class="lm" type="checkbox" style="width:24px;height:24px" id="chkSpin" '+((localStorage.getItem('opt_spin')||'1')==='1'?'checked':'')+' onchange="localStorage.setItem(\'opt_spin\',this.checked?1:0)"></div>';
  h+='<div class="field"><span class="lab" style="color:var(--good)">Aero jump</span><input class="lm" type="checkbox" style="width:24px;height:24px" id="chkJump" '+((localStorage.getItem('opt_jump')||'1')==='1'?'checked':'')+' onchange="localStorage.setItem(\'opt_jump\',this.checked?1:0)"></div>';
  h+='<div class="field"><span class="lab" style="color:var(--good)">Earth (Coriolis/Eotvos)</span><input class="lm" type="checkbox" style="width:24px;height:24px" '+((localStorage.getItem('opt_earth')||'0')==='1'?'checked':'')+' onchange="localStorage.setItem(\'opt_earth\',this.checked?1:0)"></div>';
  LIVE.forEach(function(f){
    var on=liveOn(f.live);
    h+='<div class="field"><span class="lab">'+f.l+'</span>'+
       '<input id="in_'+f.k+'" type="number" step="any" value="'+getv(f.k,f.d)+'" '+(on?'disabled':'')+' onchange="setv(\''+f.k+'\',this.value)">'+
       '<button class="lm'+(on?' live':'')+'" onclick="toggleLive(\''+f.live+'\')">'+(on?'LIVE':'MAN')+'</button></div>';
  });
  document.getElementById('loadFields').innerHTML=h;
  // profile dropdown
  var sel=document.getElementById('profileSel');
  if(sel){var names=Object.keys(loadProfiles()).sort();var act=activeName();
    var o='<option value="">-- select / unsaved --</option>';
    names.forEach(function(n){o+='<option value="'+n+'"'+(n===act?' selected':'')+'>'+n+'</option>';});
    sel.innerHTML=o;}
  syncLiveInputs();
}
function syncLiveInputs(){
  if(!data)return;
  if(liveOn('live_temp')){var e=document.getElementById('in_tempF');if(e)e.value=data.base.tempF.toFixed(1);}
  if(liveOn('live_pres')){var e=document.getElementById('in_pres');if(e)e.value=data.base.presInHg.toFixed(2);}
  if(liveOn('live_wind')){var w=avgWind();
    var e=document.getElementById('in_windmph');if(e)e.value=w.speed.toFixed(1);
    var e2=document.getElementById('in_windrel');if(e2)e2.value=Math.round(relAngle(w.dir));}
  if(liveOn('live_lat')&&data.base.gps){var e=document.getElementById('in_lat');if(e)e.value=data.base.lat.toFixed(4);}
}
function solverParams(){
  var p={};LOAD.forEach(function(f){p[f.k]=getv(f.k,f.d);});
  p.tempF = liveOn('live_temp')?data.base.tempF:getv('tempF',59);
  p.pres  = liveOn('live_pres')?data.base.presInHg:getv('pres',29.92);
  var w=avgWind();
  p.windmph = liveOn('live_wind')?w.speed:getv('windmph',10);
  p.windrel = liveOn('live_wind')?relAngle(w.dir):getv('windrel',90);
  p.lat = (liveOn('live_lat')&&data.base.gps)?data.base.lat:getv('lat',45);
  p.spin=(localStorage.getItem('opt_spin')||'1');p.jump=(localStorage.getItem('opt_jump')||'1');
  p.earth=(localStorage.getItem('opt_earth')||'0');
  return p;
}
async function doSolve(){
  if(!data)return;var p=solverParams();
  var q='/solve?mv='+p.mv+'&bc='+p.bc+'&model='+p.model+'&wgt='+p.wgt+'&cal='+p.cal+
    '&twist='+p.twist+'&blen='+p.blen+'&twistDir='+p.twistDir+'&sh='+p.sh+'&zero='+p.zero+
    '&range='+p.range+'&tempF='+p.tempF+'&pres='+p.pres+'&windmph='+p.windmph+'&windrel='+p.windrel+
    '&lat='+p.lat+'&az='+fireAz+'&earth='+p.earth+
    '&spin='+p.spin+'&jump='+p.jump;
  try{
    var r=await fetch(q,{cache:'no-store'});var s=await r.json();
    var g=document.getElementById('results');
    function c(k,v){return '<div class="card"><div class="k">'+k+'</div><div class="big">'+v+'</div></div>';}
    if(!s.ok){g.innerHTML='<div class="card">Solver could not converge - check inputs.</div>';return;}
    // Zero offset = WHERE THE GROUP LANDED vs point of aim. Correct against it:
    //   group HIGH  -> hold LOWER (less come-up)   group LOW   -> more come-up
    //   group RIGHT -> hold LEFT                    group LEFT  -> hold right
    var emag=parseFloat(getv('zeroElevMag',0))||0, edir=getv('zeroElevDir','high');
    var wmag=parseFloat(getv('zeroWindMag',0))||0, wdir=getv('zeroWindDir','right');
    var ze=(edir==='high')?-emag:emag;
    var zw=(wdir==='right')?-wmag:wmag;
    var eMOA=s.elevMOA+ze, eMil=s.elevMil+ze/3.4377;
    var wMOA=s.windMOA+zw, wMil=s.windMil+zw/3.4377;
    var offNote=(emag||wmag)?'<div class="card" style="grid-column:1/3;text-align:center;color:var(--dim);font-size:.72rem">group offset applied: '+emag.toFixed(2)+' MOA '+edir+', '+wmag.toFixed(2)+' MOA '+wdir+'</div>':'';
    g.innerHTML=
      c('Elevation',eMOA.toFixed(1)+' MOA')+c('&nbsp;',eMil.toFixed(2)+' mil')+
      c('Windage',Math.abs(wMOA).toFixed(1)+' '+(wMOA>=0?'R':'L')+' MOA')+
      c('&nbsp;',Math.abs(wMil).toFixed(2)+' '+(wMil>=0?'R':'L')+' mil')+
      c('Spin drift',s.spinDriftIn.toFixed(1)+' in')+c('Aero jump',s.aeroJumpMOA.toFixed(2)+' MOA')+
      c('TOF',s.tof.toFixed(2)+' s')+c('Vel @ tgt',Math.round(s.vRemain)+' fps')+
      c('SG',s.sg.toFixed(2))+c('Drop',s.dropIn.toFixed(1)+' in')+offNote;
  }catch(e){document.getElementById('results').innerHTML='<div class="card">Solve failed.</div>';}
}
function maybeAutoSolve(){
  if(view!=='solver')return;
  if(liveOn('live_temp')||liveOn('live_pres')||liveOn('live_wind'))doSolve();
}

async function poll(){
  try{
    var r=await fetch('/data.json',{cache:'no-store'});data=await r.json();
    var s=document.getElementById('status');s.textContent='live';s.className='status live';
    if(view!=='avg'&&view!=='pred'&&view!=='solver'&&!(data.nodes||[]).some(function(n){return n.id==view;}))view=(section==='forecast')?'pred':'avg';
    render();maybeAutoSolve();
  }catch(e){var s=document.getElementById('status');s.textContent='offline';s.className='status off';}
}
setSection(section);saveAz();drawWheel();poll();setInterval(poll,350);
</script></body></html>
)HTMLPAGE";

class WebUI {
private:
    WebServer server{80};
    String (*jsonProvider)() = nullptr;
    String (*solveProvider)(WebServer&) = nullptr;
    String (*sensProvider)(WebServer&) = nullptr;
public:
    void begin(const char* ssid, const char* pass,
               String (*jprov)(), String (*sprov)(WebServer&),
               String (*senprov)(WebServer&) = nullptr) {
        jsonProvider = jprov; solveProvider = sprov; sensProvider = senprov;
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid, pass);
        IPAddress ip = WiFi.softAPIP();
        server.on("/", [this]() { server.send_P(200, "text/html", WEBUI_PAGE); });
        server.on("/data.json", [this]() {
            server.sendHeader("Cache-Control","no-store");
            server.send(200, "application/json", jsonProvider ? jsonProvider() : "{}");
        });
        server.on("/solve", [this]() {
            server.sendHeader("Cache-Control","no-store");
            server.send(200, "application/json", solveProvider ? solveProvider(server) : "{}");
        });
        server.on("/sensitivity", [this]() {
            server.sendHeader("Cache-Control","no-store");
            server.send(200, "application/json", sensProvider ? sensProvider(server) : "{\"ok\":false}");
        });
        server.onNotFound([this]() { server.send_P(200, "text/html", WEBUI_PAGE); });
        server.begin();
        Serial.printf("[WebUI] AP '%s'  ->  http://%s\n", ssid, ip.toString().c_str());
    }
    void handle() { server.handleClient(); }
};

#endif // WEBUI_H
