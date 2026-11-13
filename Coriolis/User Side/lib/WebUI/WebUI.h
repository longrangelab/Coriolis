#ifndef WEBUI_H
#define WEBUI_H

/**
 * WebUI.h  --  phone dashboard hosted by the receiver
 * ===================================================
 * WiFi AP + web server. Phone joins the AP and opens http://192.168.4.1.
 * Tabs: AVG, each node, and SOLVER (ballistic calculator).
 *
 * Ballistics run on the PHONE: /engine.js (EngineJs.h) is a JavaScript port of
 * the Skyward Ballistics engine. The ESP32 solver (Ballistics.h) still answers
 * /solve and /sensitivity, but the page only uses them if /engine.js failed to
 * load -- every solve on the ESP32 stalls the radio loop for 100-400 ms.
 *
 * GEAR (rifles, loads, bullets) is kept in Skyward Ballistics' library format
 * (gear.js / GearJs.h) and stored on THIS box's flash, so every phone that joins
 * sees the same gear and it survives a phone clearing its browser data:
 *   GET  /gear        the stored library file (gzip or plain JSON); 204 if none
 *   GET  /gear/info   {"ok","ver","bytes","gz","max","fs"}
 *   POST /gear?base=<ver>&gz=0|1   multipart upload replacing the file; refused
 *                     with 409 if someone else saved since <ver> (phone re-merges)
 * The file lives in the 896 KB data partition of huge_app.csv (LittleFS), which
 * a normal reflash of the merged .bin does not touch.
 *
 * The SOLVER tab opens /sky.js (SkyUiJs.h): Skyward Ballistics' calculator,
 * gear, solution/table/target card/calibrate and hit-probability screens, full
 * screen over the page. It uses the same gear, live wind and group offsets as
 * the rest of the page; the classic form is under More -> Classic Coriolis solver.
 *
 * FLASH: WiFi + web server is large -> use "Huge APP" partition if it overflows.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include <sys/stat.h>
#include "EngineJs.h"
#include "GearJs.h"
#include "SkyUiJs.h"

// Largest gear file we accept. The old file stays on flash until the new one is
// complete, so two copies must fit in the 896 KB partition with room to spare.
#define GEAR_PATH "/gear.bin"     // the library file as the phone sent it
#define GEAR_TMP  "/gear.tmp"     // upload in progress
#define GEAR_META "/gear.meta"    // "<version> <gzip 0|1>"
#ifndef GEAR_MAX_BYTES
#define GEAR_MAX_BYTES (384UL * 1024UL)
#endif

static const char WEBUI_PAGE[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Wind Lab</title><meta name="theme-color" content="#01274A">
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
  .tab.on{background:var(--fire);color:var(--onFire);border-color:var(--fire)}
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
  .azVal{font-family:var(--mono);color:var(--fireInk);font-size:1.4rem;font-weight:700}
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
  .solveBtn{width:100%;padding:16px;margin-top:12px;background:var(--fire);color:var(--onFire);
    border:none;border-radius:12px;font-weight:800;font-size:1.1rem;letter-spacing:.05em}
  .res{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}
  .res .card{text-align:center}
  .res .big{color:var(--fireInk)}
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
  /* session logging + analysis */
  .logbar{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:12px;margin:0 0 12px}
  .logbtns{display:flex;gap:8px}
  .logbtns .step{padding:14px 6px}
  .logstat{margin-top:10px;text-align:center;font-family:var(--mono);font-size:.8rem;color:var(--dim)}
  .recdot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--warn);margin-right:6px;
    vertical-align:middle;animation:blink 1s steps(2,start) infinite}
  @keyframes blink{50%{opacity:.15}}
  button:disabled{opacity:.4}
  /* subtle press feedback on any tappable control (see wirePressFx) */
  button,.tab,.sec,.step,.solveBtn{transition:transform .08s ease}
  @keyframes btnPress{0%{transform:scale(1)}45%{transform:scale(.955)}100%{transform:scale(1)}}
  .pressed{animation:btnPress .13s ease-out}
  .stbl{width:100%;border-collapse:collapse;font-family:var(--mono);font-size:.74rem}
  .stbl th{color:var(--dim);font-weight:700;text-align:left;padding:6px 6px;border-bottom:1px solid var(--line);white-space:nowrap}
  .stbl td{padding:6px 6px;border-bottom:1px solid var(--line);vertical-align:middle}
  .stbl input,.stbl select{width:100%;min-width:56px;padding:6px;background:var(--bg);color:var(--ink);
    border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.78rem}
  .stbl .num{width:64px}
  .tblwrap{overflow-x:auto;-webkit-overflow-scrolling:touch}
  .statline{color:var(--ink);font-family:var(--mono);font-size:.86rem;margin:4px 0}
  .statline b{color:var(--fireInk)}
  .chartbox{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:12px;margin-top:12px}
  .chartbox svg{max-width:none}
  .rawmodal{position:fixed;inset:0;background:rgba(0,0,0,.7);display:flex;align-items:center;justify-content:center;padding:16px;z-index:50}
  .rawmodal textarea{width:100%;height:60vh;background:var(--bg);color:var(--ink);border:1px solid var(--line);
    border-radius:10px;font-family:var(--mono);font-size:.72rem;padding:10px}
  .card2{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:16px;margin-top:10px}
  .card2 h2{margin:0 0 10px 0;font-size:1rem}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .fRow{display:flex;align-items:center;justify-content:space-between;gap:10px;margin:6px 0}
  .fRow label{font-size:.82rem;color:var(--dim)}
  .fRow input{width:120px;padding:10px;background:var(--bg2b,#151a22);color:var(--ink);
    border:1px solid var(--line);border-radius:8px;text-align:right}
  #sbNodeTbl th{font-weight:600;padding:4px 3px}
  #sbNodeTbl td{padding:3px}
  .sbi{width:56px;padding:6px 4px;background:var(--bg2b,#151a22);color:var(--ink);
    border:1px solid var(--line);border-radius:6px;text-align:right;font-size:.8rem}
  /* ===== Skyward look: same colours, toolbar, logo and bottom bar as the Solver (sky.js) ===== */
  :root{--bg:#FCF9F5;--panel:#FFFFFF;--line:#CCD3DB;--ink:#01274A;--dim:#3A4D66;
    --fire:#01D1FF;--onFire:#01274A;--fireInk:#005F80;--accentStroke:#01274A;--wind:#0A6FB0;--good:#1B7F4B;--warnY:#8A5E00;--warn:#B00020;
    --bg2b:#F8F4EE;--plot:#01274A;--toolbar:#01274A;--onToolbar:#FCF9F5;--cyan:#01D1FF;
    --mono:Roboto,"Google Sans",system-ui,-apple-system,"Segoe UI",sans-serif}
  :root[data-theme=dark]{--bg:#04182E;--panel:#0E2F52;--line:#2B4666;--ink:#FCF9F5;--dim:#C5D0DC;
    --fire:#01D1FF;--onFire:#01274A;--fireInk:#5CDDFF;--accentStroke:#01D1FF;--wind:#7FE3FF;--good:#7BE0A6;--warnY:#FFD23F;--warn:#FFB4AB;
    --bg2b:#071F3A;--plot:#03111F}
  body{font-family:Roboto,"Google Sans",system-ui,-apple-system,"Segoe UI",sans-serif;font-variant-numeric:tabular-nums;padding-bottom:96px}
  header{position:sticky;top:0;z-index:60;margin:-14px -14px 6px;padding:0 10px 0 14px;min-height:64px;border:0;
    background:var(--toolbar);color:var(--onToolbar)}
  .brand{display:flex;align-items:center;gap:12px;font-size:20px;font-weight:700;letter-spacing:.14em}
  .wlLogo{position:relative;width:34px;height:34px;flex:0 0 auto}
  .wlLogo>*{position:absolute;inset:0;width:100%;height:100%;max-width:none;margin:0}
  .wlLogo .rif{animation:wlSpin 24s linear infinite}
  @keyframes wlSpin{to{transform:rotate(360deg)}}
  @media (prefers-reduced-motion:reduce){.wlLogo .rif{animation:none}}
  .status{padding:6px 12px;border-radius:14px;background:rgba(255,255,255,.1);color:#C5D0DC;font-family:inherit;font-weight:700}
  .status.live{color:#7BE0A6}.status.off{color:#FFB4AB}
  .sections{position:fixed;left:0;right:0;bottom:0;z-index:55;margin:0;gap:0;background:var(--panel);
    border-top:1px solid var(--line);padding:10px 0 calc(12px + env(safe-area-inset-bottom))}
  .sec{display:flex;flex-direction:column;align-items:center;gap:4px;padding:0;border:0;border-radius:0;background:transparent;
    color:var(--dim);font-weight:500;letter-spacing:0;font-size:13px;cursor:pointer}
  .sec .ind{width:60px;height:32px;border-radius:16px;display:flex;align-items:center;justify-content:center;transition:background .18s}
  .sec .ind svg{width:26px;height:26px;max-width:none;margin:0;fill:currentColor}
  .sec.on{background:transparent;color:var(--ink);font-weight:700;border:0}
  .sec.on .ind{background:var(--cyan);color:#01274A}
  h2{display:flex;align-items:center;color:var(--ink);font-size:15px;font-weight:700;letter-spacing:.06em;margin:24px 0 8px}
  h2::before{content:"";flex:0 0 auto;width:5px;height:20px;background:var(--cyan);margin-right:8px}
  .card2 h2{margin:0 0 10px}
  .hero,.wheelWrap,.card2,.chartbox,.logbar{border-radius:16px}
  .tab{border-radius:24px;border:1.5px solid var(--line);min-height:48px}
  .tab.on{background:var(--cyan);color:#01274A;border-color:var(--cyan)}
  .solveBtn{border-radius:28px;border:1.5px solid var(--accentStroke);color:var(--onFire);min-height:56px}
  .step{border-radius:12px;background:var(--panel)}
  .field input,.field select{background:var(--panel)}
  .lm.live{color:#FCF9F5}
  :root[data-theme=dark] .lm.live{color:#00140a}
</style>
<script>
// Light or dark, the same as the Solver's Settings -> Colours (default: same as the phone).
function wlTheme(){var t='system';try{t=(JSON.parse(localStorage.getItem('skySettings'))||{}).theme||'system';}catch(e){}
  var dark=t==='dark'||(t!=='light'&&!!(window.matchMedia&&matchMedia('(prefers-color-scheme: dark)').matches));
  document.documentElement.setAttribute('data-theme',dark?'dark':'light');}
wlTheme();
try{matchMedia('(prefers-color-scheme: dark)').addEventListener('change',wlTheme);}catch(e){}
</script></head><body><div class="wrap">

<header><div class="brand"><span class="wlLogo" id="wlLogo"><svg class="bore" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100"><defs><clipPath id="wlc"><circle cx="50" cy="50" r="23.6"/></clipPath></defs><circle cx="50" cy="50" r="23.6" fill="#01D1FF"/><g clip-path="url(#wlc)" fill="none" stroke="#FFFFFF" stroke-width="3.6" stroke-linecap="round"><path d="M24,43.5H55.5a5.2,5.2 0 1 0 -5.2,-5.2"/><path d="M24,51.5H67"/><path d="M24,59.5H51.5a5.2,5.2 0 1 1 -5.2,5.2"/></g></svg></span>WIND LAB</div><div id="status" class="status off">connecting</div></header>
<div class="sections">
  <div id="secBasic" class="sec on" onclick="setSection('basic')"><span class="ind"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4,10h11.5c1.9,0 3.5,-1.6 3.5,-3.5S17.4,3 15.5,3c-1.6,0 -2.9,1 -3.3,2.5l1.9,0.6c0.2,-0.6 0.8,-1.1 1.4,-1.1 0.8,0 1.5,0.7 1.5,1.5S16.3,8 15.5,8H4V10zM4,12v2h14.5c0.8,0 1.5,0.7 1.5,1.5S19.3,17 18.5,17c-0.6,0 -1.2,-0.4 -1.4,-1l-1.9,0.6c0.5,1.4 1.8,2.4 3.3,2.4 1.9,0 3.5,-1.6 3.5,-3.5S20.4,12 18.5,12H4z"/></svg></span>Basic</div>
  <div id="secFore" class="sec" onclick="setSection('forecast')"><span class="ind"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M3.5,18.49l6,-6.01 4,4L22,6.92l-1.41,-1.41 -7.09,7.97 -4,-4L2,16.99z"/></svg></span>Forecast</div>
  <div id="secSand" class="sec" onclick="setSection('sandbox')"><span class="ind"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M19.8,18.4L14,10.67V6.5l1.35,-1.69C15.61,4.48 15.38,4 14.96,4H9.04C8.62,4 8.39,4.48 8.65,4.81L10,6.5v4.17L4.2,18.4C3.71,19.06 4.18,20 5,20h14c0.82,0 1.29,-0.94 0.8,-1.6z"/></svg></span>Sandbox</div>
  <div id="secLog" class="sec" onclick="setSection('log')"><span class="ind"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M3,13h2v-2H3v2zm0,4h2v-2H3v2zm0,-8h2V7H3v2zm4,4h14v-2H7v2zm0,4h14v-2H7v2zM7,7v2h14V7H7z"/></svg></span>Log</div>
  <div id="secSolver" class="sec" onclick="setView('solver')"><span class="ind"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M12,2C6.48,2 2,6.48 2,12s4.48,10 10,10 10,-4.48 10,-10S17.52,2 12,2zM13,19.93V17h-2v2.93C7.38,19.48 4.52,16.62 4.07,13H7v-2H4.07C4.52,7.38 7.38,4.52 11,4.07V7h2V4.07c3.62,0.45 6.48,3.31 6.93,6.93H17v2h2.93c-0.45,3.620 -3.31,6.48 -6.93,6.93zM12,10.5c-0.83,0 -1.5,0.67 -1.5,1.5s0.67,1.5 1.5,1.5 1.5,-0.67 1.5,-1.5 -0.67,-1.5 -1.5,-1.5z"/></svg></span>Solver</div>
</div>
<div id="tabs" class="tabs"></div>

<!-- FLEET SLEEP (BASIC + FORECAST sections) -->
<div id="sleepBar" class="hide" style="display:flex;align-items:center;gap:10px;margin:2px 0 12px 0;
     padding:10px 12px;border:1px solid var(--line);border-radius:12px">
  <div id="sleepInfo" style="flex:1;font-size:.8rem;color:var(--dim)"></div>
  <button id="rewakeBtn" onclick="rewakeFleet()" class="hide" style="flex:0 0 auto;padding:10px 16px;
     border-radius:10px;border:2px solid var(--fireInk);background:transparent;
     color:var(--fireInk);font-weight:800;letter-spacing:.04em;margin-right:6px"></button>
  <button id="sleepBtn" onclick="toggleFleetSleep()" style="flex:0 0 auto;padding:10px 16px;
     border-radius:10px;border:1px solid var(--fire);background:var(--fire);
     color:var(--onFire);font-weight:800;letter-spacing:.04em"></button>
</div>

<!-- ======================= SANDBOX (virtual BC lab) ======================= -->
<div id="sandboxView" class="hide">
  <!-- sub-tabs: SETUP (15 nodes + BC inputs) / SIMULATION (monte carlo) -->
  <div class="tabs" style="padding-top:0">
    <div id="sbTabSetup" class="tab on" onclick="sbSetSub('setup')">SETUP</div>
    <div id="sbTabSim"   class="tab"    onclick="sbSetSub('sim')">SIMULATION</div>
  </div>

  <!-- ---------- SETUP SUBTAB ---------- -->
  <div id="sbSetup">
    <div class="card2">
      <h2>Virtual profile</h2>
      <div style="font-size:.78rem;color:var(--dim);margin:-4px 0 8px">
        Load a rifle saved in the Solver, or edit inputs below. All computation is local &mdash; nothing is sent to the receiver.
      </div>
      <select id="sbProfileSel" onchange="sbLoadProfile(this.value)" style="width:100%;padding:12px;
        background:var(--bg2b,#151a22);color:var(--ink);border:1px solid var(--line);border-radius:10px"></select>
      <div id="sbLoadInputs"></div>
    </div>

    <div class="card2">
      <h2>Downrange geometry</h2>
      <div class="fRow"><label>Target range (yd)</label><input id="sbRange" type="number" value="500" oninput="sbRecalc();sbInvalidateCurve()"></div>
      <div class="fRow"><label>Segment width (yd)</label><input id="sbSegYd" type="number" value="25" oninput="sbRecalc()"></div>
      <div style="font-size:.76rem;color:var(--dim);margin-top:4px">
        Enter each node's distance from the muzzle. The segmented model weights muzzle wind far more heavily than target-side wind.
      </div>
    </div>

    <div class="card2">
      <div style="display:flex;justify-content:space-between;align-items:center">
        <h2 style="margin:0">Wind nodes</h2>
        <div>
          <button class="step" onclick="sbAutoDistribute()">Auto-space</button>
          <button class="step" onclick="sbAllOn(true)">All on</button>
          <button class="step" onclick="sbAllOn(false)">All off</button>
        </div>
      </div>
      <div style="overflow-x:auto">
        <table id="sbNodeTbl" style="width:100%;border-collapse:collapse;font-size:.8rem;margin-top:6px"></table>
      </div>
    </div>

    <div class="card2">
      <h2>Wind call (this instant)</h2>
      <div class="grid2">
        <div class="card"><div class="k">Weighted call</div><div class="v" id="sbCallMoa">--</div></div>
        <div class="card"><div class="k">Drift</div><div class="v" id="sbCallIn">--</div></div>
        <div class="card"><div class="k">Active nodes</div><div class="v" id="sbActive">--</div></div>
        <div class="card"><div class="k">Effective wind</div><div class="v" id="sbEffWind">--</div></div>
      </div>
    </div>
  </div>

  <!-- ---------- SIMULATION SUBTAB ---------- -->
  <div id="sbSim" class="hide">
    <div class="card2">
      <h2>Monte Carlo</h2>
      <div class="fRow"><label>Target size (in)</label><input id="sbTgt" type="number" value="10" oninput="sbRunSim()"></div>
      <div class="fRow"><label>Runs</label>
        <select id="sbRuns" onchange="sbRunSim()" style="padding:10px;background:var(--bg2b,#151a22);color:var(--ink);border:1px solid var(--line);border-radius:8px">
          <option>100</option><option>250</option><option>500</option><option>1000</option>
        </select>
      </div>
      <div style="font-size:.76rem;color:var(--dim)">
        Each run perturbs every active node's speed &amp; direction by its SD, recomputes the drift, and checks it against a target centered on your wind call.
      </div>
    </div>

    <div class="card2">
      <div class="grid2">
        <div class="card"><div class="k">Hit probability</div><div class="v" id="sbHitPct" style="color:var(--fireInk)">--</div></div>
        <div class="card"><div class="k">90% cone</div><div class="v" id="sbCone">--</div></div>
      </div>
      <canvas id="sbTargetCv" width="320" height="320" style="width:100%;max-width:340px;display:block;margin:10px auto;background:var(--plot);border:1px solid var(--line);border-radius:12px"></canvas>
      <div style="text-align:center;font-size:.76rem;color:var(--dim)">Impact dispersion vs target (horizontal = wind miss)</div>
    </div>

    <div class="card2">
      <div style="display:flex;justify-content:space-between;align-items:center;gap:8px">
        <h2 style="margin:0">Where wind matters</h2>
        <button id="sbPopulateBtn" class="step" onclick="sbPopulateCurve()" disabled>Populate</button>
      </div>
      <div style="font-size:.76rem;color:var(--dim);margin:6px 0 6px">
        Sensitivity per downrange segment (inches of drift per mph). Taller = a meter there has more influence on the call.
      </div>
      <div id="sbPopulateHint" style="font-size:.72rem;color:var(--warnY);margin:0 0 8px">
        Select a saved rifle profile above, then press Populate to compute the true curve.
      </div>
      <canvas id="sbSensCv" width="320" height="170" style="width:100%;background:var(--plot);border:1px solid var(--line);border-radius:12px"></canvas>
      <div id="sbCurveSrc" style="text-align:center;font-size:.72rem;color:var(--dim);margin-top:6px">--</div>
      <canvas id="sbHistCv" width="320" height="150" style="width:100%;background:var(--plot);border:1px solid var(--line);border-radius:12px;margin-top:10px"></canvas>
      <div style="text-align:center;font-size:.76rem;color:var(--dim)">Miss distribution across all runs</div>
    </div>
  </div>

  <!-- session save/load/export -->
  <div class="card2">
    <h2>Session</h2>
    <div style="display:flex;gap:8px;flex-wrap:wrap">
      <button class="step" onclick="sbSaveSession()">Save</button>
      <button class="step" onclick="sbLoadSessionPrompt()">Load</button>
      <button class="step" onclick="sbExportSession()">Export .json</button>
      <button class="step" onclick="document.getElementById('sbImportFile').click()">Import .json</button>
    </div>
    <input type="file" id="sbImportFile" accept="application/json,.json" style="display:none" onchange="sbImportSession(this)">
    <select id="sbSessionSel" onchange="sbLoadSession(this.value)" style="width:100%;margin-top:8px;padding:10px;
      background:var(--bg2b,#151a22);color:var(--ink);border:1px solid var(--line);border-radius:8px"></select>
  </div>
</div>


<!-- STABILITY (Forecast / Prediction tab only) -->
<div id="stabView" class="hide">
  <h2 id="stabTitle">Stability</h2>
  <div id="logBar" class="logbar">
    <div class="logbtns">
      <button id="btnStart" class="step" onclick="startLog()">&#9679; START</button>
      <button id="btnStop" class="step" onclick="stopLog()" disabled>&#9632; STOP</button>
      <button id="btnMark" class="step" onclick="markShot()" disabled>&#9670; MARK</button>
    </div>
    <div id="logStat" class="logstat">not recording</div>
  </div>
  <div id="sentBadge" class="stabBadge bg-warn hide" style="margin-bottom:10px"><div class="word">--</div><div class="sub"></div></div>
  <div id="windHold" class="win" style="margin-bottom:10px"><div class="wt">WIND HOLD</div><div class="wv">--</div></div>
  <!-- AIMBOT: drives the windage stepper to the hold above. Sits directly
       under WIND HOLD so it rides along in both BASIC (avg) and FORECAST (pred). -->
  <div id="aimRow" class="hide" style="display:flex;gap:8px;margin-bottom:10px">
    <button id="aimBtn" onclick="aimGo()" style="flex:2;padding:14px;border-radius:12px;
      border:1px solid var(--fire);background:var(--fire);color:var(--onFire);font-weight:800;
      letter-spacing:.06em;font-size:1rem">AIMBOT</button>
    <button id="zeroBtn" onclick="aimZero()" style="flex:1;padding:14px;border-radius:12px;
      border:1px solid var(--line);background:var(--bg);color:var(--ink);font-weight:800;
      letter-spacing:.06em">ZERO</button>
  </div>
  <div id="stabBadge" class="stabBadge bg-warn"><div class="word">--</div><div class="sub"></div></div>
  <div class="grid" style="grid-template-columns:1fr 1fr 1fr 1fr;margin-top:10px" id="stabWindows"></div>
  <div id="stabThPanel" class="wheelWrap" style="margin-top:14px"></div>
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
  <div id="nodeSleepRow" class="hide" style="margin-top:14px"></div>
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
      <button class="step" onclick="exportProfiles()">Export gear</button>
      <button class="step" onclick="document.getElementById('importFile').click()">Import gear</button>
    </div>
    <input type="file" id="importFile" accept="application/json,.json" style="display:none" onchange="importProfiles(this)">
    <div style="font-size:.74rem;color:var(--dim);margin-top:8px">
      Gear uses the <b>Skyward Ballistics</b> library format: import a Skyward export (rifles, loads or bullets)
      or an old Coriolis <i>rifle_profiles.json</i>. Saved on the base station, so every phone sees it.
    </div>
    <div id="gearStatus" style="font-size:.74rem;color:var(--dim);margin-top:6px"></div>
  </div>
  <h2>Bullet</h2>
  <div class="wheelWrap">
    <select id="bulletSel" onchange="pickBullet(this.value)" style="width:100%;padding:12px;
      background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:10px;
      font-family:var(--mono);font-size:.9rem"></select>
    <label style="display:flex;align-items:center;gap:8px;font-size:.78rem;color:var(--dim);margin-top:8px">
      <input type="checkbox" id="bulletAll" onchange="renderGearLists()" style="width:20px;height:20px"> show every caliber</label>
  </div>
  <h2>Ballistic solver</h2>
  <div class="wheelWrap">
    <div id="loadFields"></div>
    <button class="solveBtn" onclick="doSolve()">Solve</button>
    <div class="res" id="results"></div>
    <details id="tableBox" style="margin-top:10px" ontoggle="if(this.open)renderTable()"><summary style="cursor:pointer;color:var(--dim)">Come-up table</summary>
      <div class="stepRow" style="margin-top:8px"><label style="flex:1;color:var(--dim);font-size:.78rem">Step (yd)
        <input id="tblStep" type="number" value="100" min="10" step="10" onchange="renderTable()" style="width:100%"></label></div>
      <div id="tableOut" style="overflow-x:auto;font-family:var(--mono);font-size:.78rem"></div></details>
    <div class="warn">Verify against a known solver and on paper before trusting for real dope. Standard G1/G7 curves unless the bullet carries its own drag curve.</div>
  </div>
</div>

<div id="wheelBlock">
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

</div><!-- /wheelBlock -->

<!-- SESSION LOG + ANALYSIS -->
<div id="logView" class="hide"></div>

<footer id="base">Base station: waiting&hellip;</footer>
</div>
<script src="/engine.js"></script>
<script src="/gear.js"></script>
<script src="/sky.js"></script>
<script>
// The rifling ring of the logo comes with sky.js (the same artwork as the Solver's).
(function(){var b=document.getElementById('wlLogo');if(!b||!window.SkyUI||!SkyUI.logo)return;
  var i=new Image();i.className='rif';i.alt='';i.src=SkyUI.logo.rifLight;b.insertBefore(i,b.firstChild);})();
</script>
<script>
var data=null, view='avg', dataAt=0;
// The base stopped answering (phone left its Wi-Fi, base off): the last reading is
// not live wind. Not while the poll is paused on purpose (Sandbox).
function liveStale(){return !!(data&&pollTimer&&Date.now()-dataAt>12000);}
var pollTimer=null;   // handle for the 350 ms poll; Sandbox suspends it
var section=localStorage.getItem('section')||'basic';
if(section==='forecast')view='pred';
var fireAz=parseInt(localStorage.getItem('fireAz')||'0',10)||0;

// ---- load profile (all manual) + live-capable environment inputs ----
var LOAD=[
 {k:'mv',l:'Muzzle vel (fps)',d:2700},{k:'bc',l:'BC',d:0.243},
 {k:'model',l:'Drag model',d:7,opt:[[7,'G7'],[1,'G1']]},
 {k:'bcBands',l:'BC bands (bc@fps, ...)',d:'',t:'text',ph:'blank = one BC'},
 {k:'dragCurve',l:'Drag curve (Mach:Cd, ...)',d:'',t:'area',ph:'for Custom curve: 0.5:0.142, 0.9:0.150, 1.0:0.290, ...'},
 {k:'wgt',l:'Weight (gr)',d:175},{k:'cal',l:'Caliber (in)',d:0.308},
 {k:'twist',l:'Twist (in/turn)',d:10},{k:'blen',l:'Bullet length (in)',d:1.24},
 {k:'twistDir',l:'Twist dir',d:1,opt:[[1,'Right'],[-1,'Left']]},
 {k:'sh',l:'Sight height (in)',d:1.75},{k:'zero',l:'Zero (yd)',d:100},
 {k:'range',l:'Target range (yd)',d:500},
 {k:'zeroElevMag',l:'Group off. elev (MOA)',d:0},
 {k:'zeroElevDir',l:'Group landed (elev)',d:'high',opt:[['high','High'],['low','Low']]},
 {k:'zeroWindMag',l:'Group off. wind (MOA)',d:0},
 {k:'zeroWindDir',l:'Group landed (wind)',d:'right',opt:[['right','Right'],['left','Left']]},
 {k:'powderTempF',l:'MV measured at (F)',d:59},
 {k:'mvPerF',l:'MV change (fps per F)',d:0},
 {k:'dragScale',l:'Drag scale (from truing)',d:1}
];
// live-capable: temp, pressure, wind speed, wind angle
var LIVE=[
 {k:'tempF',l:'Temperature (F)',d:59,live:'live_temp'},
 {k:'pres',l:'Pressure (inHg)',d:29.92,live:'live_pres'},
 {k:'windmph',l:'Wind speed (mph)',d:10,live:'live_wind'},
 {k:'windrel',l:'Wind angle (rel)',d:90,live:'live_wind'},
 {k:'lat',l:'Latitude (deg)',d:45,live:'live_lat'}
];
// getv: read a profile field. localStorage.getItem returns null when the key
// is missing, but various historical code paths and old saved profiles can
// leave a key present with an EMPTY string value -- treat that the same as
// missing, otherwise empty strings get passed to /solve and .toFloat()/.toInt()
// on the receiver silently converts them to 0 (breaking twistDir -> zero spin
// drift + aero jump, model -> silent G7 fallback, etc).
function getv(k,d){var v=localStorage.getItem('sol_'+k);return (v===null||v==='')?d:v;}
function setv(k,v){localStorage.setItem('sol_'+k,v);}
function liveOn(key){return (localStorage.getItem(key)||'1')==='1';}
function toggleLive(key){localStorage.setItem(key,liveOn(key)?'0':'1');renderSolver();}

// ---- rifle profiles = gear (rifles / loads / bullets, Skyward format) ----
// The fields above are a working copy of ONE saved load. Gear (gear.js) keeps
// the records in Skyward Ballistics' library format, on this phone and on the
// base station's flash, and translates to/from these flat fields.
// NOT captured in a profile: target range, temp/pressure/wind, Live/Manual
// toggles, and (since gear moved to Skyward's format) the spin/jump toggles,
// which are now plain settings like in Skyward.
function profileFields(){return LOAD.map(function(f){return f.k;}).filter(function(k){return k!=='range';});}
function haveGear(){if(window.Gear)return true;alert('The gear script did not load. Reload the page.');return false;}
function activeLoadId(){var id=localStorage.getItem('activeLoadId')||'';return (window.Gear&&Gear.load(id))?id:'';}
function setActiveId(id){localStorage.setItem('activeLoadId',id||'');}
function activeName(){var id=activeLoadId();return id?Gear.label(Gear.load(id)):'';}
// label -> flat profile, for code that lists or reads profiles by name
function loadProfiles(){var o={};if(!window.Gear)return o;
  Gear.loads().forEach(function(l){var f=Gear.flat(l.id);f.loadId=l.id;o[Gear.label(l)]=f;});return o;}
function snapshot(){var p={};LOAD.forEach(function(f){if(f.k!=='range')p[f.k]=getv(f.k,f.d);});
  p.bulletId=getv('bulletId','');return p;}
function applyProfile(p){
  profileFields().forEach(function(k){if(p[k]!==undefined)setv(k,p[k]);});
  setv('bulletId',p.bulletId||'');
  if(p.loadId)stampLoad(p.loadId);
}
// The solver reads the form fields (sol_*). Remember which saved version of the
// active load they came from, so an edit synced in from another phone or the
// Skyward app replaces them instead of being silently ignored.
function stampLoad(id){var l=window.Gear&&Gear.load(id);localStorage.setItem('sol_loadStamp',l?(id+'@'+(l.updatedAt||0)):'');}
function followActiveLoad(){
  var id=activeLoadId();if(!id)return false;
  var l=Gear.load(id),st=localStorage.getItem('sol_loadStamp')||'';
  if(st===id+'@'+(l.updatedAt||0))return false;
  var fresh=(getv('mv',null)===null);   // fields never filled (e.g. just migrated)
  if(!fresh&&st.indexOf(id+'@')!==0){stampLoad(id);return false;}  // older page: adopt as-is
  var f=Gear.flat(id);f.loadId=id;applyProfile(f);
  if(!fresh&&!window.__skyQuiet)flash('"'+activeName()+'" was updated on another device');
  return true;
}
function saveProfile(name,asNew){
  if(!haveGear())return false;
  var snap=snapshot(),from=activeLoadId()||null,rifleAll=false;
  var sh=Gear.rifleShare(snap,from,!!asNew);
  if(sh)rifleAll=confirm('You changed twist or sight height. '+sh.others+' other load(s) use the rifle "'+sh.name+'".\n\n'+
    'OK = change the rifle for all of them\nCancel = give this load its own rifle');
  try{var id=Gear.saveFlat(name,snap,from,!!asNew,rifleAll);
    setActiveId(id);setv('bulletId',Gear.load(id).bulletId);stampLoad(id);renderSolver();return true;
  }catch(e){alert('Not saved: '+e.message);return false;}
}
function saveActive(){var l=activeLoadId()&&Gear.load(activeLoadId());if(!l){saveAsProfile();return;}
  if(saveProfile(l.name,false))flash('Saved "'+activeName()+'"');}
function saveAsProfile(){if(!haveGear())return;var l=activeLoadId()&&Gear.load(activeLoadId());
  var n=prompt('Profile name:',l?l.name:'My rifle');if(n&&n.trim()&&saveProfile(n.trim(),true))flash('Saved "'+activeName()+'"');}
function renameProfile(){var id=activeLoadId();if(!id)return;var l=Gear.load(id);
  var nn=prompt('Rename profile:',l.name);if(!nn||!nn.trim()||nn.trim()===l.name)return;
  try{Gear.saveFlat(nn.trim(),Gear.flat(id),id,false);stampLoad(id);}catch(e){alert('Not renamed: '+e.message);}renderSolver();}
function deleteProfile(){var id=activeLoadId();if(!id)return;if(!confirm('Delete profile "'+activeName()+'"?'))return;
  Gear.removeLoad(id);setActiveId('');renderSolver();}
function loadProfile(id){if(!id||!window.Gear)return;var f=Gear.flat(id);if(!f)return;
  f.loadId=id;applyProfile(f);setActiveId(id);renderSolver();doSolve();}
function newProfile(){
  if(!confirm('Start a new rifle profile?\nCurrent fields will reset to defaults. Saved profiles are not affected.'))return;
  profileFields().forEach(function(k){localStorage.removeItem('sol_'+k);});   // getv() falls back to LOAD defaults
  localStorage.removeItem('sol_bulletId');
  setActiveId('');                                                           // dropdown -> "-- select / unsaved --"
  renderSolver();
  flash('New profile - enter rifle data, then Save As');
}
// Pick a bullet from the library: copies its data into the fields.
function pickBullet(id){
  if(!id||!window.Gear)return;var b=Gear.bullet(id);if(!b)return;
  var rows=(b.bcPoints||[]).slice().sort(function(x,y){return y.minVelFps-x.minVelFps;});
  setv('cal',b.diameterIn);setv('wgt',b.weightGr);if(b.lengthIn>0)setv('blen',b.lengthIn);
  setv('model',b.dragModel==='G1'?1:(b.dragModel==='CUSTOM'?'C':7));
  setv('bc',rows.length?rows[0].bc:'');
  setv('bcBands',rows.length>1?rows.map(function(x){return (+x.bc)+'@'+(+(x.minVelFps||0));}).join(', '):'');
  setv('dragCurve',Gear.curveToText(b.dragCurve));
  setv('bulletId',id);
  renderSolver();flash('Bullet: '+(b.maker?b.maker+' ':'')+b.name+' - Save to keep it');
}
function gearDate(){return new Date().toISOString().slice(0,10);}
function exportProfiles(){
  if(!haveGear())return;
  downloadFile('coriolis_gear_'+gearDate()+'.json','application/json',Gear.exportText('master'));
}
function importProfiles(input){
  var f=input.files[0];if(!f||!haveGear())return;var r=new FileReader();
  r.onload=function(){
    try{var res=Gear.importText(r.result);renderSolver();
      flash('Imported '+res.loads+' load(s), '+res.bullets+' bullet(s), '+res.rifles+' rifle(s)');
    }catch(e){alert('Import failed: '+e.message);}
  };
  r.readAsText(f);input.value='';
}
function renderGearStatus(){
  var el=document.getElementById('gearStatus');if(!el||!window.Gear)return;
  var m=Gear.meta(),L=Gear.lib();
  var txt=L.loads.length+' load(s), '+L.bullets.length+' bullet(s)';
  var st=m.status||'checking base station\u2026';
  el.innerHTML=txt+' &middot; <span style="color:'+(m.err?'var(--warnY)':'var(--good)')+'">'+st+'</span>'+(m.err?' ('+m.err+')':'');
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
function nodeWeights(){try{var w=JSON.parse(localStorage.getItem('nodeWeights')||'{}');return w&&typeof w==='object'&&!Array.isArray(w)?w:{};}catch(e){return {};}}
function storeWeights(w){localStorage.setItem('nodeWeights',JSON.stringify(w));}
function configuredNodeWeight(w,id,count){
  var v=w[id];
  if(v===undefined||v===null||v===''||!isFinite(Number(v)))return 100/count;
  return Math.max(0,Number(v));
}
// Effective shares are derived from currently wind-valid nodes. Keep saved
// multi-node weights intact when nodes leave/rejoin; a sole valid node always
// supplies 100%. An all-zero configuration falls back to equal shares.
function effectiveNodeWeights(){
  var ns=((data&&data.nodes)||[]).filter(function(n){return n.windOk!==false;});
  var w=nodeWeights(),pct={},total=0;
  ns.forEach(function(n){total+=configuredNodeWeight(w,n.id,ns.length);});
  ns.forEach(function(n){pct[n.id]=(ns.length===1||total<=0)?100/ns.length:
    configuredNodeWeight(w,n.id,ns.length)/total*100;});
  return {nodes:ns,pct:pct,total:total,equalFallback:ns.length>1&&total<=0};
}
// per-node speed multiplier (e.g. a meter at max ordinate sped up for winds aloft)
function nodeMults(){try{return JSON.parse(localStorage.getItem('nodeMult')||'{}');}catch(e){return {};}}
function storeMults(m){localStorage.setItem('nodeMult',JSON.stringify(m));}
function getMult(id){var m=nodeMults();var v=(m[id]!==undefined)?parseFloat(m[id]):1;return (isFinite(v)&&v>0)?v:1;}
function setNodeMult(id,v){var m=nodeMults();var f=parseFloat(v);m[id]=(isFinite(f)&&f>0)?f:1;storeMults(m);render();}
function resetMultsOne(){var ns=(data&&data.nodes)||[];var m={};ns.forEach(function(n){m[n.id]=1;});storeMults(m);renderWeightsPanel(true);render();}
function weightedAvgWind(){
  var all=(data&&data.nodes&&!liveStale())?data.nodes:[];
  // Drop nodes whose anemometer is dead or whose reading is too old (both arrive
  // as windOk:false from the receiver, which applies the same test to its own average).
  // They stay visible on their own tab -- they just don't vote on the wind call.
  // Legacy receivers don't send windOk at all; undefined means "trust it", so
  // this degrades to the old behaviour rather than zeroing everything out.
  if(liveStale())return{speed:0,dir:0,speedSd:0,dirSd:0,count:0,bad:0,stale:true};
  var weights=effectiveNodeWeights(),ns=weights.nodes;
  var nBad=all.length-ns.length;
  if(!ns.length)return{speed:0,dir:0,speedSd:0,dirSd:0,count:0,bad:nBad};
  // Average the wind as VECTORS (speed x direction), not speed and direction
  // separately. Averaging them separately invents wind: 10 mph from 0 deg plus
  // 10 mph from 180 deg nets to zero, but a plain speed average (10) glued to a
  // unit-vector direction (noise, ~90 deg) reported a full 10 mph crosswind.
  // The weighted vector mean gives exactly the weighted mean crosswind and
  // headwind, which is what the solver needs.
  var sumSpd=0,sumSpdSd=0,sumDirSd=0,sx=0,sy=0;
  ns.forEach(function(n){
    var f=weights.pct[n.id]/100, v=n.speed*getMult(n.id);
    sumSpd+=f*v;sumSpdSd+=f*n.speedSd*getMult(n.id);sumDirSd+=f*n.dirSd;
    var r=n.dir*Math.PI/180;sx+=f*v*Math.cos(r);sy+=f*v*Math.sin(r);
  });
  var speed=Math.sqrt(sx*sx+sy*sy);
  var dir=Math.atan2(sy,sx)*180/Math.PI;if(dir<0)dir+=360;
  // agree: 1 = every meter blows the same way; near 0 = they cancel out. Low
  // agreement means the meters see different winds and the net call is small
  // because they disagree, not because it's calm.
  var agree=sumSpd>0?speed/sumSpd:1;
  return{speed:speed,dir:dir,speedSd:sumSpdSd,dirSd:sumDirSd,count:ns.length,bad:nBad,
         scalarSpeed:sumSpd,agree:agree};
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
async function autoWeights(tries){
  if(!data){flash('no data yet');return;}
  var usable=effectiveNodeWeights().nodes.length;
  if(usable<2){renderWeightsPanel(true);flash(usable?'Only one valid wind node: 100% weight':'No valid wind readings');return;}
  if(!activeName()){flash('pick a rifle profile first');return;}
  tries=tries||0;
  var p=solverParams();
  if(engineLocal()){autoWeightsLocal(p);return;}
  var q='/sensitivity?mv='+p.mv+'&bc='+p.bc+'&model='+p.model+'&wgt='+p.wgt+'&cal='+p.cal+
    '&twist='+p.twist+'&blen='+p.blen+'&sh='+p.sh+'&zero='+p.zero+'&range='+p.range+
    '&tempF='+p.tempF+'&pres='+p.pres+'&az='+fireAz;
  var man=nodeDownManual();
  Object.keys(man).forEach(function(id){q+='&dr'+id+'='+man[id];});
  try{
    var r=await fetch(q,{cache:'no-store'});var s=await r.json();
    if(s.computing){
      // Same device compute as the sandbox chart; long ranges take a while.
      if(tries===0)flash('computing sensitivity curve\u2026');
      if(tries<SB_POLL_MAX)setTimeout(function(){autoWeights(tries+1);},SB_POLL_MS);
      else flash('sensitivity timed out \u2014 long ranges are slow to compute');
      return;
    }
    if(!s.ok){flash('sensitivity: solver could not converge');return;}
    // Node availability may change while the receiver computes the curve.
    if(effectiveNodeWeights().nodes.length<2){renderWeightsPanel(true);return;}
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
// Phone-side version of the receiver's /sensitivity weighting (same math as
// main.cpp handleSensitivity + Sensitivity.h): one crosswind sweep, then each
// meter's raw sensitivity at its downrange spot, normalized to 100%.
function autoWeightsLocal(p){
  var curve=Gear.sensitivity(p,p.range);
  if(!curve.ok){flash('sensitivity: '+(curve.err||'solver could not converge'));return;}
  if(effectiveNodeWeights().nodes.length<2){renderWeightsPanel(true);return;}
  var b=(data&&data.base)||{},baseFix=!!(b.lat&&b.lon),man=nodeDownManual(),rows=[],sum=0;
  ((data&&data.nodes)||[]).forEach(function(nd){
    var dy=-1,src='none';
    if(man[nd.id]!==undefined){dy=+man[nd.id];src='manual';}
    else if(baseFix&&nd.gps!==false&&nd.lat&&nd.lon){dy=SkyEngine.projectDownrangeYd(b.lat,b.lon,fireAz,nd.lat,nd.lon);src='gps';}
    var r=(src==='none')?0:SkyEngine.sensitivityAt(curve,dy);
    rows.push({id:nd.id,downYd:dy,src:src,r:r});sum+=r;
  });
  lastSens={};var w=nodeWeights(),none=0;
  rows.forEach(function(nd){
    var pct=sum>0?nd.r/sum*100:0;
    lastSens[nd.id]={downYd:nd.downYd,src:nd.src,wPct:pct};
    if(nd.src==='none')none++;
    w[nd.id]=Math.round(pct*10)/10;
  });
  storeWeights(w);renderWeightsPanel(true);render();
  flash('weights set from ballistics'+(none?(' - '+none+' node(s) need a position'):''));
}
var weightsBuiltFor='';
function renderWeightsPanel(force){
  var panel=document.getElementById('weightsPanel');if(!panel)return;
  var ns=(data&&data.nodes)||[];
  if((view!=='avg'&&view!=='pred')||!ns.length){panel.className='wheelWrap hide';return;}
  panel.className='wheelWrap';
  var weights=effectiveNodeWeights(),single=weights.nodes.length===1;
  var sig=ns.map(function(n){return n.id+':'+(n.windOk!==false);}).sort().join(',');
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
      var valid=n.windOk!==false;
      var locked=single||!valid;
      var v=!valid?0:single?100:configuredNodeWeight(w,n.id,weights.nodes.length);
      var mv=(m[n.id]!==undefined)?m[n.id]:1;
      var dv=(dman[n.id]!==undefined)?dman[n.id]:'';
      h+='<div style="margin:10px 0;padding:10px;background:var(--bg);border:1px solid var(--line);border-radius:10px">'+
         '<div style="color:var(--dim);font-size:.85rem;margin-bottom:8px">Node '+n.id+' <span id="wlbl_'+n.id+'">('+n.speed.toFixed(1)+' mph)</span></div>'+
         '<div style="display:flex;gap:10px">'+
         '<label style="flex:1;color:var(--dim);font-size:.72rem">Weight %<input type="number" step="any" min="0" '+(locked?'disabled ':'')+'style="'+istyle+'" value="'+v+'" onchange="setNodeWeight('+n.id+',this.value)"></label>'+
         '<label style="flex:1;color:var(--dim);font-size:.72rem">Multiplier &times;<input type="number" step="0.05" min="0" style="'+istyle+'" value="'+mv+'" onchange="setNodeMult('+n.id+',this.value)"></label>'+
         '</div>'+
         ((!valid)?'<div style="color:var(--dim);font-size:.72rem;margin-top:6px">No valid wind data; excluded from the average.</div>':
          single?'<div style="color:var(--dim);font-size:.72rem;margin-top:6px">Only valid wind node: 100% weight. Speed multiplier remains adjustable.</div>':'')+
         '<div style="display:flex;gap:10px;margin-top:8px"><label style="flex:1;color:var(--dim);font-size:.72rem">Downrange (yd) <span style="opacity:.7">blank = GPS</span>'+
         '<input type="number" step="any" min="0" style="'+istyle+'" value="'+dv+'" onchange="setNodeDown('+n.id+',this.value)"></label></div>'+
         '<div id="slbl_'+n.id+'" style="color:var(--dim);font-size:.72rem;margin-top:6px">'+sensReadout(n.id)+'</div>'+
         '</div>';
    });
    h+='<div class="stepRow"><button class="step" '+(weights.nodes.length<2?'disabled ':'')+'onclick="autoWeights()">Auto from ballistics</button>'+
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
  var weights=effectiveNodeWeights(),ns=weights.nodes;
  if(!ns.length){e.textContent='No valid wind readings';return;}
  if(ns.length===1){e.textContent='Node '+ns[0].id+': 100% of the average';return;}
  var shares=ns.map(function(n){return 'Node '+n.id+': '+weights.pct[n.id].toFixed(1)+'%';}).join(' / ');
  e.textContent=(weights.equalFallback?'All weights are zero; using equal shares. ':'Effective weights: ')+shares;
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
    'Label which meter each node is running. Shown on the node\'s tab.</div>';
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
// stabTh is the AGGREGATE badge threshold (GO/CAUTION/WAIT) for the big Forecast
// badge. (The former per-node "meter quality" thresholds have been removed.)
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

// (Per-meter-type "meter quality" SD thresholds were removed. The meter TYPE
// labeling above is kept so you can see which meter each node is running; the
// per-node GOOD/NOISY quality badge and its editable thresholds are gone.)
// ---- solver request throttling ---------------------------------------------
// render() runs on every 350 ms poll, and on the Prediction tab it called
// updateWindHold() each time -- one full /solve per poll. On the receiver a
// solve is ~100-400 ms of software double-precision math that BLOCKS the TDMA
// loop, so the phone was keeping the base station too busy to coordinate its
// own radio network: dropped node packets, late beacons, nodes losing sync.
// The irony is that this happened hardest on the tab you actually shoot from.
//
// Two guards, both needed:
//   solveBusy  -- never have more than one request in flight. Without this, a
//                 slow response just queues up more requests behind it and the
//                 backlog grows without bound.
//   MIN_MS     -- a floor between requests. 2 s is far faster than wind
//                 conditions or your hold changes, and cuts the load ~6x.
// force=true bypasses both, for user actions that must feel immediate.
var solveBusy=false, windHoldLast=0;
// ---- aimbot (windage stepper) ----
var aimHoldMOA=0;        // latest signed windage hold in MOA (set by updateWindHold)
var aimHoldOk=false;     // false when the last hold had no valid live wind behind it
var aimBusy=false;
async function aimGo(){
  if(aimBusy)return;
  if(!data||!data.aim||!data.aim.present){flash('no stepper connected');return;}
  if(!aimHoldOk){flash('no valid wind -- not dialing');return;}
  aimBusy=true;
  var btn=document.getElementById('aimBtn'), old=btn.textContent;
  btn.textContent='MOVING\u2026';
  try{
    var r=await fetch('/aim?op=go&moa='+aimHoldMOA.toFixed(3),{cache:'no-store'});
    var s=await r.json();
    if(!s.ok)flash('aimbot: '+(s.err||'failed'));
    else if(s.clamped)flash('hold clamped at turret limit');
  }catch(e){flash('aimbot failed');}
  finally{aimBusy=false;btn.textContent=old;}
}
async function aimZero(){
  if(aimBusy)return;
  if(!data||!data.aim||!data.aim.present){flash('no stepper connected');return;}
  if(!confirm('Return the windage turret to its power-on zero?'))return;
  aimBusy=true;
  var btn=document.getElementById('zeroBtn'), old=btn.textContent;
  btn.textContent='\u2026';
  try{
    var r=await fetch('/aim?op=zero',{cache:'no-store'});
    var s=await r.json();
    if(!s.ok)flash('zero: '+(s.err||'failed'));
  }catch(e){flash('zero failed');}
  finally{aimBusy=false;btn.textContent=old;}
}
// Show the Aimbot/Zero row only where WIND HOLD shows (avg + pred) AND only
// when the controller is actually present. Absent -> hidden, so nobody taps a
// dead button.
function renderAimRow(){
  var row=document.getElementById('aimRow');if(!row)return;
  var show=data&&data.aim&&data.aim.present&&(view==='avg'||view==='pred')&&activeName();
  row.className=show?'':'hide';
  if(show)row.style.display='flex';
  if(show){
    var btn=document.getElementById('aimBtn');
    if(data.aim.moving){btn.textContent='MOVING\u2026';}
    else if(!aimBusy){btn.textContent=aimHoldOk?('AIMBOT '+(aimHoldMOA>=0?'R':'L')+' '+Math.abs(aimHoldMOA).toFixed(1)):'AIMBOT -- no wind';}
  }
}
// The receiver solver needed ~2 s between solves; the phone engine takes a few
// ms, so it can follow the live wind closely (see minMs below).
var WINDHOLD_MIN_MS=2000, AUTOSOLVE_MIN_MS=2000;
function minMs(ms){return engineLocal()?500:ms;}

async function updateWindHold(force){
  var el=document.getElementById('windHold');
  if(!el)return;
  if(!data){el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">--</div>';return;}
  if(!activeName()){aimHoldOk=false;el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">-- (no rifle profile)</div>';return;}
  if(liveWindMissing()){aimHoldOk=false;el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv" style="color:var(--warn)">NO VALID WIND METER</div>';return;}
  if(!force){
    if(solveBusy)return;
    if(Date.now()-windHoldLast<minMs(WINDHOLD_MIN_MS))return;
  }
  solveBusy=true; windHoldLast=Date.now();
  try{
    var p=solverParams();
    var s=await solveP(p);
    // The aimbot may only dial the hold that is on screen. A solver error or a
    // failed request shows no hold, so the previous one must not stay armed.
    if(!s.ok){aimHoldOk=false;el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">solver error</div>';return;}
    var wmag=parseFloat(getv('zeroWindMag',0))||0, wdir=getv('zeroWindDir','right');
    var zw=(wdir==='right')?-wmag:wmag;
    var wMOA=s.windMOA+zw, wMil=s.windMil+zw/3.4377;
    // Remember the exact signed hold the stepper should dial. Sign convention:
    // wMOA>=0 means hold RIGHT; the ATtiny's DIR_INVERT sorts out which way that
    // physically turns. We send signed MOA and let the controller do clicks.
    aimHoldMOA=wMOA; aimHoldOk=true;
    el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">'+Math.abs(wMOA).toFixed(1)+' '+(wMOA>=0?'R':'L')+
      ' MOA &middot; '+Math.abs(wMil).toFixed(2)+' '+(wMil>=0?'R':'L')+' mil</div>';
  }catch(e){aimHoldOk=false;el.innerHTML='<div class="wt">WIND HOLD</div><div class="wv">--</div>';
  }finally{solveBusy=false;windHoldLast=Date.now();}
}
function renderStability(){
  if(!data||!data.stability)return;
  var s=data.stability, cur=tier(s.cur.spdSd,s.cur.dirSd);
  // Zero valid meters means zero SDs, which used to read as a green GO.
  if(!data.avg||!data.avg.count) cur={c:'warn',lbl:'NO VALID METERS',go:'NO WIND'};
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
  renderSentBadge();
  renderSentPanel();
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
    if(s.windOk===false||n.windOk===false){missing++;return;}   // dead meter can't 'agree'
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

function avgWind(){if(!data)return{ok:false,speed:0,dir:0};var a=weightedAvgWind();return{ok:a.count>0,speed:a.speed,dir:a.dir};}
// Live wind is switched on but no meter has valid wind: there is NO wind call.
// Every consumer must say so instead of solving with 0 mph (a confident, wrong hold).
function liveWindMissing(){return liveOn('live_wind')&&!avgWind().ok;}
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

// ============================================================================
// SESSION LOGGING + ANALYSIS  (client-side; localStorage; no firmware changes)
// ============================================================================
// Records a timestamped wind + wind-hold time series while you shoot a string.
// START/STOP/MARK live on the Prediction tab (where you watch wind); the LOG
// section reviews the session, lets you enter each shot's miss distance, and
// gives a *rough* correlation between the wind call and the observed impact.
// Every sample carries: weighted-average wind, per-node wind, the live wind
// hold from the solver, stability, sentinel agreement, base atmosphere and
// density altitude. Marks additionally carry the shot's hold + your miss entry.
//
// Honesty note baked into the UI: at low shot counts, and with shooter/rifle
// error plus the solver's known hot-elevation bias mixed in, "correlation" is
// indicative, not gospel. Bias +/- spread is the trustworthy headline; the
// regression / implied-wind numbers are secondary and caveated.

function milPerIn(range){return range>0?1000/(range*36):0;}   // mil per inch at target
function moaPerIn(range){return range>0?(10800/Math.PI)/(range*36):0;}
function fmtDur(s){s=Math.max(0,Math.floor(s));var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),ss=s%60;
  function p(n){return (n<10?'0':'')+n;}
  return (h>0?(h+':'):'')+p(m)+':'+p(ss);}
// Density altitude (ft), standard approximation from station pressure + temp.
function daFt(tempF,presInHg){
  if(!(presInHg>0))return null;
  var pa=(29.92-presInHg)*1000.0;                 // pressure altitude, ft
  var isaC=15.0-1.98*(pa/1000.0);                 // ISA temp at that PA
  var oatC=(tempF-32.0)*5.0/9.0;
  return pa+118.8*(oatC-isaC);
}

function newLogState(){return {recording:false,startedAt:0,samples:[],marks:[],shotSeq:0,
  profileName:'',profileParams:null,lastHold:null,nSinceSave:0,recovered:false};}
function logGet(){try{var s=JSON.parse(localStorage.getItem('windlog'));
  if(s&&s.recording){s.recording=false;s.recovered=true;}   // page died mid-record -> recover as stopped
  return s;}catch(e){return null;}}
function saveLog(){try{localStorage.setItem('windlog',JSON.stringify(logState));}catch(e){}}
var logState=logGet()||newLogState();
var logTimer=null, logBusy=false;
var logCadence=parseInt(localStorage.getItem('logCadence')||'1000',10)||1000;

function snapshotProfileParams(){var p={};LOAD.forEach(function(f){p[f.k]=getv(f.k,f.d);});
  LIVE.forEach(function(f){p[f.k]=getv(f.k,f.d);});return p;}

// Ask the receiver for the current wind hold, applying the same group-offset
// ("dialed") math the solver screen uses. Returns null if no profile / no fix.
async function solveNow(){
  if(!data||!activeName())return null;
  if(liveWindMissing())return null;          // no wind call -> nothing to log
  var p=solverParams();
  try{
    var s=await solveP(p);if(!s.ok)return null;
    var wmag=parseFloat(getv('zeroWindMag',0))||0,wdir=getv('zeroWindDir','right');
    var emag=parseFloat(getv('zeroElevMag',0))||0,edir=getv('zeroElevDir','high');
    var zw=(wdir==='right')?-wmag:wmag, ze=(edir==='high')?-emag:emag;
    return {windMil:s.windMil,windMOA:s.windMOA,elevMil:s.elevMil,elevMOA:s.elevMOA,
      dialedWindMil:s.windMil+zw/3.4377,dialedWindMOA:s.windMOA+zw,
      dialedElevMil:s.elevMil+ze/3.4377,dialedElevMOA:s.elevMOA+ze,
      tof:s.tof,vRemain:s.vRemain,sg:s.sg,range:parseFloat(p.range)||0};
  }catch(e){return null;}
}

// Current weighted-wind reading, synchronous (no network) — shared by the
// per-tick snapshot and the instant part of markShot().
function curWtd(){
  var a=weightedAvgWind();
  var rel=relAngle(a.dir);
  var ns=(data&&data.nodes)||[];
  var gust=0;ns.forEach(function(n){if(n.gust>gust)gust=n.gust;});
  return {spd:a.speed,dir:a.dir,relDir:rel,count:a.count,
    cross:a.speed*Math.sin(rel*Math.PI/180), head:a.speed*Math.cos(rel*Math.PI/180),
    spdSd:a.speedSd,dirSd:a.dirSd,gust:gust};
}
// Build one full sample from the current live data + a hold object.
// NOTE: named logSample(), not snapshot() -- there's a pre-existing,
// unrelated snapshot() (no args, builds a rifle-profile object for saving)
// higher up in this file. Two top-level functions with the same name would
// silently collide (the later one wins), which is exactly what happened
// here until this rename -- saveProfile() was calling THIS function instead
// of the profile one, so profile Save was silently storing wind-log data
// instead of rifle data. If you're touching this file, keep these two
// distinct.
function logSample(hold){
  var w=curWtd();
  var ns=(data&&data.nodes)||[];
  var base=data?data.base:{tempF:0,presInHg:0,battPct:0};
  var st=(data&&data.stability)?data.stability:null;
  var sent=null;
  if(sentinelEnabled()){var ev=evalSentinel();var ag=ev.live.filter(function(p){return p.agree;}).length;
    sent={agree:ag,live:ev.live.length,missing:ev.missing,frac:ev.live.length?ag/ev.live.length:null};}
  return {
    t:Date.now(), tRel:logState.startedAt?((Date.now()-logState.startedAt)/1000):0,
    nCount:w.count, az:fireAz,
    wtd:w,
    da:daFt(base.tempF,base.presInHg),
    base:{tempF:base.tempF,presInHg:base.presInHg,battPct:base.battPct},
    stab: st?{tier:tier(st.cur.spdSd,st.cur.dirSd).go,curSpdSd:st.cur.spdSd,curDirSd:st.cur.dirSd,
              w60SpdSd:st.w60.spdSd,w60DirSd:st.w60.dirSd}:null,
    sent:sent, hold:hold,
    nodes: ns.map(function(n){return {id:n.id,spd:n.speed,dir:n.dir,gust:n.gust,spdSd:n.speedSd,dirSd:n.dirSd,
      tempF:n.tempF,presInHg:n.presInHg,battV:n.battV,rssi:n.rssi,age:n.age,lat:n.lat,lon:n.lon};})
  };
}

async function logTick(){
  if(!logState.recording||!data){return;}
  if(logBusy)return; logBusy=true;
  try{
    var hold=await solveNow(); if(hold)logState.lastHold=hold;
    logState.samples.push(logSample(hold||logState.lastHold||null));
    logState.nSinceSave=(logState.nSinceSave||0)+1;
    if(logState.nSinceSave>=20){saveLog();logState.nSinceSave=0;}
    renderLogBar();
    if(section==='log'){updateLogStatus();renderCharts();}
  }finally{logBusy=false;}
}
function startTimer(){stopTimer();logTimer=setInterval(logTick,logCadence);}
function stopTimer(){if(logTimer){clearInterval(logTimer);logTimer=null;}}
function setCadence(v){logCadence=parseInt(v,10)||1000;localStorage.setItem('logCadence',logCadence);
  if(logState.recording)startTimer();flash('log interval '+(logCadence/1000)+' s');}

function startLog(){
  if(logState.recording)return;
  if(logState.samples.length||logState.marks.length){
    if(!confirm('Start a new session? This clears the last one. Export it first if you want to keep it.'))return;
  }
  logState=newLogState();
  logState.recording=true; logState.startedAt=Date.now();
  logState.profileName=activeName(); logState.profileParams=snapshotProfileParams();
  saveLog(); startTimer(); logTick(); renderLogBar();
  if(section==='log')renderLog();
  flash('recording started');
}
function stopLog(){
  if(!logState.recording)return;
  logState.recording=false; stopTimer(); saveLog(); renderLogBar();
  if(section==='log')renderLog();
  flash('stopped: '+logState.samples.length+' samples, '+logState.marks.length+' shots');
}
// MARK is instant: timestamp, shot number, and current wind are all local
// data, captured synchronously the moment the button is pressed. The exact
// wind hold (which needs a /solve round-trip to the receiver) is fetched in
// the background and patched into this same shot once it arrives, so the
// button never waits on the network and the recorded shot time is the real
// moment of the press, not whenever the fetch happened to finish.
function markShot(){
  if(!logState.recording)return;
  var t=Date.now();
  var mark={shot:++logState.shotSeq, t:t,
    tRel:logState.startedAt?((t-logState.startedAt)/1000):0,
    wtd:curWtd(), az:fireAz, hold:logState.lastHold,
    range:(logState.lastHold&&logState.lastHold.range)||(logState.profileParams&&parseFloat(logState.profileParams.range))||0,
    conv:'applied', missH_in:null, missV_in:null, note:''};
  logState.marks.push(mark);
  saveLog(); renderLogBar();
  if(section==='log')renderLog();
  flash('shot '+mark.shot+' marked');
  solveNow().then(function(hold){patchMarkHold(mark,hold);});
}
// Patch a freshly-solved hold into an already-recorded mark, in place —
// targets the one table cell by id rather than rebuilding the shot table,
// so it can never steal focus from a miss-distance field mid-edit.
function patchMarkHold(mark,hold){
  if(!hold)return;
  logState.lastHold=hold;
  mark.hold=hold;
  if(!mark.range)mark.range=hold.range;
  saveLog();
  if(section!=='log')return;
  var i=logState.marks.indexOf(mark);
  if(i<0)return;
  var hc=document.getElementById('hold_'+i);
  if(hc)hc.textContent=Math.abs(hold.windMil).toFixed(2)+(hold.windMil>=0?'R':'L')+' mil';
  updateShotComputed();
}

function delMark(i){if(!confirm('Delete shot '+(logState.marks[i]&&logState.marks[i].shot)+'?'))return;
  logState.marks.splice(i,1);saveLog();renderLog();}
function setMiss(i,field,v){var m=logState.marks[i];if(!m)return;
  var f=parseFloat(v);m[field]=(v===''||isNaN(f))?null:f;saveLog();updateShotComputed();}
function setConv(i,v){var m=logState.marks[i];if(!m)return;m.conv=v;saveLog();updateShotComputed();}
function setNote(i,v){var m=logState.marks[i];if(!m)return;m.note=v;saveLog();}

function renderLogBar(){
  var b1=document.getElementById('btnStart'),b2=document.getElementById('btnStop'),
      b3=document.getElementById('btnMark'),st=document.getElementById('logStat');
  if(!b1)return;
  var rec=logState.recording;
  b1.disabled=rec; b2.disabled=!rec; b3.disabled=!rec;
  b2.className='step'+(rec?' live':'');
  if(!st)return;
  if(rec){var el=Math.floor((Date.now()-logState.startedAt)/1000);
    st.innerHTML='<span class="recdot"></span>REC '+fmtDur(el)+' &middot; '+logState.samples.length+
      ' samples &middot; '+logState.marks.length+' shots';
  }else if(logState.samples.length||logState.marks.length){
    st.innerHTML=(logState.recovered?'recovered':'stopped')+' &middot; '+logState.samples.length+
      ' samples, '+logState.marks.length+' shots';
  }else{st.textContent='not recording';}
}

// ---- correlation math (pure; host-tested) ----------------------------------
// For each shot with an entered horizontal miss we form a residual in mil:
//   conv 'applied' (you dialed the app's wind hold): residual = miss
//   conv 'center'  (you held point of aim, no wind hold): residual = miss + hold.windMil
//     (expected impact when holding center is -hold.windMil, so residual is the
//      difference between where it landed and where the call said it would).
// + miss/residual = right; + elevation miss = high.
function computeStats(marks){
  var H=[],X=[],V=[],IMP=[];
  marks.forEach(function(m){
    if(!m.hold||!m.range)return;
    var mpi=milPerIn(m.range);
    if(m.missH_in!==null&&m.missH_in!==undefined&&!isNaN(m.missH_in)){
      var missH=m.missH_in*mpi;
      var resH=(m.conv==='center')?(missH+m.hold.windMil):missH;
      H.push(resH); X.push(m.hold.windMil);
      var cw=m.wtd?m.wtd.cross:0;
      if(Math.abs(cw)>0.5&&Math.abs(m.hold.windMil)>0.02){
        var mpmph=Math.abs(m.hold.windMil)/Math.abs(cw); IMP.push(resH/mpmph);
      }
    }
    if(m.missV_in!==null&&m.missV_in!==undefined&&!isNaN(m.missV_in)){
      V.push(m.missV_in*mpi);
    }
  });
  function mean(a){return a.reduce(function(s,x){return s+x;},0)/a.length;}
  function sd(a){if(a.length<2)return null;var mu=mean(a);
    var v=a.reduce(function(s,x){return s+(x-mu)*(x-mu);},0)/(a.length-1);return Math.sqrt(v);}
  var out={nH:H.length,nV:V.length};
  if(H.length){out.windBias=mean(H);out.windSd=sd(H);}
  if(V.length){out.elevBias=mean(V);out.elevSd=sd(V);}
  if(IMP.length){out.impliedWind=mean(IMP);out.nImp=IMP.length;}
  if(H.length>=3){
    var n=H.length,sx=0,sy=0,sxx=0,sxy=0,syy=0;
    for(var i=0;i<n;i++){sx+=X[i];sy+=H[i];sxx+=X[i]*X[i];sxy+=X[i]*H[i];syy+=H[i]*H[i];}
    var dxx=sxx-sx*sx/n, dxy=sxy-sx*sy/n, dyy=syy-sy*sy/n;
    if(dxx>1e-9){var slope=dxy/dxx,intercept=(sy-slope*sx)/n;
      var r=(dyy>1e-12)?dxy/Math.sqrt(dxx*dyy):null;
      out.reg={slope:slope,intercept:intercept,r:r,r2:(r!==null?r*r:null),n:n};}
  }
  out.pts=marks.map(function(m){                          // for the scatter chart
    if(!m.hold||!m.range||m.missH_in===null||m.missH_in===undefined||isNaN(m.missH_in))return null;
    return {x:m.hold.windMil, y:m.missH_in*milPerIn(m.range), shot:m.shot};
  }).filter(function(p){return p;});
  return out;
}

// ---- analysis view ---------------------------------------------------------
function logShellHtml(){
  return '<div id="logShell">'+
    '<h2>Session</h2>'+
    '<div class="wheelWrap">'+
      '<div id="logSummary" class="statline" style="color:var(--dim)">--</div>'+
      '<div class="field" style="margin-top:10px"><span class="lab">Log interval</span>'+
        '<select onchange="setCadence(this.value)">'+
          [500,1000,2000,5000].map(function(ms){return '<option value="'+ms+'"'+(ms===logCadence?' selected':'')+'>'+(ms/1000)+' s</option>';}).join('')+
        '</select></div>'+
      '<div class="stepRow"><button class="step" onclick="exportJson()">Export JSON</button>'+
        '<button class="step" onclick="exportSummaryCsv()">Summary CSV</button>'+
        '<button class="step" onclick="exportNodesCsv()">Nodes CSV</button></div>'+
      '<div class="stepRow"><button class="step" onclick="startLog()">New session</button></div>'+
    '</div>'+
    '<h2>Shots &amp; miss entry</h2>'+
    '<div class="wheelWrap">'+
      '<div style="color:var(--dim);font-size:.76rem;margin-bottom:8px">Enter where each shot landed '+
        'relative to your aim point, in inches at the target (+ right / + high). Pick whether you '+
        '<b>dialed</b> the app\'s wind hold or <b>held center</b>.</div>'+
      '<div class="tblwrap"><div id="shotTable"></div></div>'+
    '</div>'+
    '<h2>Wind call vs. observed impact</h2>'+
    '<div class="wheelWrap"><div id="logStats"></div></div>'+
    '<div id="logCharts"></div>'+
    '<div id="rawHost"></div>'+
  '</div>';
}
function renderLog(){
  var el=document.getElementById('logView');if(!el)return;
  if(!document.getElementById('logShell'))el.innerHTML=logShellHtml();
  updateLogStatus();
  renderShotTable(false);
  renderLogStats();
  renderCharts();
}
function updateLogStatus(){
  var e=document.getElementById('logSummary');if(!e)return;
  var dur=logState.startedAt?(((logState.recording?Date.now():(logState.samples.length?logState.samples[logState.samples.length-1].t:logState.startedAt))-logState.startedAt)/1000):0;
  var prof=logState.profileName||'(no profile)';
  var txt=(logState.recording?'RECORDING':'stopped')+' &middot; '+prof+' &middot; '+
    fmtDur(dur)+' &middot; '+logState.samples.length+' samples &middot; '+logState.marks.length+' shots';
  e.innerHTML=txt;
}

var shotSig='';
function renderShotTable(force){
  var host=document.getElementById('shotTable');if(!host)return;
  var sig=logState.marks.map(function(m){return m.shot;}).join(',');
  if(force||sig!==shotSig){
    shotSig=sig;
    if(!logState.marks.length){
      host.innerHTML='<div style="color:var(--dim);font-size:.8rem">No shots yet. Press MARK on the Prediction tab as each shot breaks.</div>';
      return;
    }
    var h='<table class="stbl"><thead><tr>'+
      '<th>#</th><th>time</th><th>wind</th><th>hold</th><th>held</th>'+
      '<th>miss L/R (in)</th><th>miss U/D (in)</th><th>note</th><th>resid</th><th></th></tr></thead><tbody>';
    logState.marks.forEach(function(m,i){
      var hold=m.hold?(Math.abs(m.hold.windMil).toFixed(2)+(m.hold.windMil>=0?'R':'L')+' mil'):'--';
      var wclk=m.wtd?(m.wtd.spd.toFixed(1)+' @'+(function(){var hr=Math.round(relAngle(m.wtd.dir)/30)%12;return hr===0?12:hr;})()+"o'c"):'--';
      h+='<tr>'+
        '<td>'+m.shot+'</td>'+
        '<td>'+fmtDur(m.tRel)+'</td>'+
        '<td>'+wclk+'</td>'+
        '<td id="hold_'+i+'">'+hold+'</td>'+
        '<td><select onchange="setConv('+i+',this.value)">'+
          '<option value="applied"'+(m.conv==='applied'?' selected':'')+'>dialed</option>'+
          '<option value="center"'+(m.conv==='center'?' selected':'')+'>center</option></select></td>'+
        '<td><input class="num" type="number" step="any" value="'+(m.missH_in===null||m.missH_in===undefined?'':m.missH_in)+'" onchange="setMiss('+i+',\'missH_in\',this.value)"></td>'+
        '<td><input class="num" type="number" step="any" value="'+(m.missV_in===null||m.missV_in===undefined?'':m.missV_in)+'" onchange="setMiss('+i+',\'missV_in\',this.value)"></td>'+
        '<td><input type="text" value="'+(m.note||'').replace(/"/g,'&quot;')+'" onchange="setNote('+i+',this.value)"></td>'+
        '<td id="resid_'+i+'" style="color:var(--dim)">--</td>'+
        '<td><button class="step" style="padding:4px 8px;font-size:.7rem" onclick="delMark('+i+')">x</button></td>'+
      '</tr>';
    });
    h+='</tbody></table>';
    host.innerHTML=h;
  }
  updateShotComputed();
}
// Update residual cells + stats + charts WITHOUT rebuilding inputs (protects
// the mobile keyboard, per the build-once rule the other panels follow).
function updateShotComputed(){
  logState.marks.forEach(function(m,i){
    var el=document.getElementById('resid_'+i);if(!el)return;
    if(!m.hold||!m.range||m.missH_in===null||m.missH_in===undefined||isNaN(m.missH_in)){el.textContent='--';return;}
    var mpi=milPerIn(m.range),missH=m.missH_in*mpi;
    var resH=(m.conv==='center')?(missH+m.hold.windMil):missH;
    el.innerHTML='<span style="color:var(--'+(Math.abs(resH)<0.15?'good':'warnY')+')">'+
      Math.abs(resH).toFixed(2)+(resH>=0?'R':'L')+'</span>';
  });
  renderLogStats();
  renderCharts();
}
function renderLogStats(){
  var el=document.getElementById('logStats');if(!el)return;
  var st=computeStats(logState.marks);
  if(st.nH<1&&st.nV<1){
    el.innerHTML='<div style="color:var(--dim);font-size:.8rem">Enter miss distances above to see how the wind call compared to where shots landed.</div>';
    return;
  }
  var h='';
  if(st.nH>=1)h+='<div class="statline">Windage: shots averaged <b>'+Math.abs(st.windBias).toFixed(2)+' mil '+(st.windBias>=0?'right':'left')+'</b> of aim'+
    (st.windSd!==null?(' (spread '+st.windSd.toFixed(2)+' mil, n='+st.nH+')'):(' (n='+st.nH+')'))+'.</div>';
  if(st.nV>=1)h+='<div class="statline">Elevation: shots averaged <b>'+Math.abs(st.elevBias).toFixed(2)+' mil '+(st.elevBias>=0?'high':'low')+'</b>'+
    (st.elevSd!==null?(' (spread '+st.elevSd.toFixed(2)+' mil, n='+st.nV+')'):(' (n='+st.nV+')'))+'.</div>';
  if(st.impliedWind!==undefined)h+='<div class="statline">Implied wind error: about <b>'+Math.abs(st.impliedWind).toFixed(1)+' mph '+(st.impliedWind>=0?'under':'over')+'-called</b> on average (n='+st.nImp+').</div>';
  if(st.reg){h+='<div class="statline" style="color:var(--dim)">Regression (miss vs. predicted hold): slope '+st.reg.slope.toFixed(2)+', intercept '+st.reg.intercept.toFixed(2)+' mil, r&sup2; '+((st.reg.r2===null?0:st.reg.r2).toFixed(2))+', n='+st.reg.n+'.</div>';
    h+='<div class="statline" style="color:var(--dim)">'+(Math.abs(st.reg.slope)>0.15?'Error grows with wind &rarr; possible calibration/scale issue.':'Error is roughly wind-independent &rarr; looks like scatter (shooter / turbulence), not a scale error.')+'</div>';
  }
  h+='<div class="warn" style="margin-top:10px">Rough, indicative only. Observed miss also contains your shooting/rifle precision and the solver&apos;s known hot-elevation bias; a handful of shots can&apos;t separate those. Treat as a trend, verify against known dope.</div>';
  el.innerHTML=h;
}

// ---- charts (hand-rolled SVG; modular, safe to remove) ---------------------
function renderCharts(){
  var host=document.getElementById('logCharts');if(!host)return;
  var samples=logState.samples, marks=logState.marks;
  var h='';
  // 1) wind speed + signed crosswind over time, with shot markers
  if(samples.length>=2){
    var W=680,Hh=200,pad=34;
    var t0=samples[0].tRel, t1=samples[samples.length-1].tRel; if(t1<=t0)t1=t0+1;
    var vals=[]; samples.forEach(function(s){vals.push(s.wtd.spd);vals.push(s.wtd.cross);});
    var vmax=Math.max.apply(null,vals.map(Math.abs)); if(!(vmax>0))vmax=1; vmax*=1.15;
    function sx(t){return pad+(t-t0)/(t1-t0)*(W-2*pad);}
    function sy(v){return Hh/2-(v/vmax)*(Hh/2-pad/2);}
    function path(sel){return samples.map(function(s,i){return (i?'L':'M')+sx(s.tRel).toFixed(1)+' '+sy(sel(s)).toFixed(1);}).join(' ');}
    var svg='<svg viewBox="0 0 '+W+' '+Hh+'" preserveAspectRatio="xMidYMid meet">'+
      '<line x1="'+pad+'" y1="'+(Hh/2)+'" x2="'+(W-pad)+'" y2="'+(Hh/2)+'" stroke="var(--line)"/>'+
      '<path d="'+path(function(s){return s.wtd.spd;})+'" fill="none" stroke="var(--wind)" stroke-width="2"/>'+
      '<path d="'+path(function(s){return s.wtd.cross;})+'" fill="none" stroke="var(--fire)" stroke-width="1.6" stroke-dasharray="4 3"/>';
    marks.forEach(function(m){var x=sx(m.tRel);svg+='<line x1="'+x.toFixed(1)+'" y1="'+(pad/2)+'" x2="'+x.toFixed(1)+'" y2="'+(Hh-pad/2)+'" stroke="var(--good)" stroke-width="1"/>'+
      '<text x="'+x.toFixed(1)+'" y="'+(pad/2-2)+'" fill="var(--good)" font-size="10" text-anchor="middle">'+m.shot+'</text>';});
    svg+='<text x="'+pad+'" y="'+(Hh-4)+'" fill="var(--dim)" font-size="10">0s</text>'+
      '<text x="'+(W-pad)+'" y="'+(Hh-4)+'" fill="var(--dim)" font-size="10" text-anchor="end">'+Math.round(t1-t0)+'s</text>'+
      '<text x="'+(pad+2)+'" y="'+(pad-4)+'" fill="var(--dim)" font-size="10">max '+vmax.toFixed(1)+'</text></svg>';
    h+='<div class="chartbox"><div class="k" style="margin-bottom:6px">Wind over time &mdash; <span style="color:var(--wind)">speed</span> / <span style="color:var(--fire)">crosswind (+R)</span> &middot; <span style="color:var(--good)">shots</span></div><div class="tblwrap">'+svg+'</div></div>';
  }
  // 2) predicted hold vs observed miss scatter + fit line
  var st=computeStats(marks);
  if(st.pts&&st.pts.length>=2){
    var W2=680,H2=220,p2=40;
    var xs=st.pts.map(function(p){return p.x;}), ys=st.pts.map(function(p){return p.y;});
    var xmin=Math.min.apply(null,xs),xmax=Math.max.apply(null,xs);
    var ymin=Math.min.apply(null,ys),ymax=Math.max.apply(null,ys);
    var xr=(xmax-xmin)||1, yr=(ymax-ymin)||1; xmin-=xr*0.1;xmax+=xr*0.1;ymin-=yr*0.1;ymax+=yr*0.1;
    function px(x){return p2+(x-xmin)/(xmax-xmin)*(W2-2*p2);}
    function py(y){return H2-p2-(y-ymin)/(ymax-ymin)*(H2-2*p2);}
    var svg2='<svg viewBox="0 0 '+W2+' '+H2+'" preserveAspectRatio="xMidYMid meet">';
    if(ymin<0&&ymax>0)svg2+='<line x1="'+p2+'" y1="'+py(0).toFixed(1)+'" x2="'+(W2-p2)+'" y2="'+py(0).toFixed(1)+'" stroke="var(--line)"/>';
    if(xmin<0&&xmax>0)svg2+='<line x1="'+px(0).toFixed(1)+'" y1="'+p2+'" x2="'+px(0).toFixed(1)+'" y2="'+(H2-p2)+'" stroke="var(--line)"/>';
    if(st.reg){var x1=xmin,x2=xmax,y1=st.reg.slope*x1+st.reg.intercept,y2=st.reg.slope*x2+st.reg.intercept;
      svg2+='<line x1="'+px(x1).toFixed(1)+'" y1="'+py(y1).toFixed(1)+'" x2="'+px(x2).toFixed(1)+'" y2="'+py(y2).toFixed(1)+'" stroke="var(--warnY)" stroke-width="1.5" stroke-dasharray="5 4"/>';}
    st.pts.forEach(function(p){svg2+='<circle cx="'+px(p.x).toFixed(1)+'" cy="'+py(p.y).toFixed(1)+'" r="4" fill="var(--wind)"/>'+
      '<text x="'+(px(p.x)+6).toFixed(1)+'" y="'+(py(p.y)+3).toFixed(1)+'" fill="var(--dim)" font-size="9">'+p.shot+'</text>';});
    svg2+='<text x="'+(W2/2)+'" y="'+(H2-6)+'" fill="var(--dim)" font-size="10" text-anchor="middle">predicted wind hold (mil, +R)</text>'+
      '<text x="12" y="'+(H2/2)+'" fill="var(--dim)" font-size="10" transform="rotate(-90 12 '+(H2/2)+')" text-anchor="middle">observed miss (mil, +R)</text></svg>';
    h+='<div class="chartbox"><div class="k" style="margin-bottom:6px">Predicted hold vs. observed miss</div><div class="tblwrap">'+svg2+'</div></div>';
  }
  host.innerHTML=h;
}

// ---- export ----------------------------------------------------------------
function csvCell(v){if(v===null||v===undefined||(typeof v==='number'&&isNaN(v)))return '';
  var s=''+v; return (/[",\n]/.test(s))?('"'+s.replace(/"/g,'""')+'"'):s;}
function csvRow(a){return a.map(csvCell).join(',');}
function isoT(ms){try{return new Date(ms).toISOString();}catch(e){return ''+ms;}}

function buildSessionJson(){
  return JSON.stringify({
    meta:{app:'Wind Lab',generatedAt:isoT(Date.now()),profile:logState.profileName,
      profileParams:logState.profileParams,startedAt:isoT(logState.startedAt),
      cadenceMs:logCadence,nSamples:logState.samples.length,nShots:logState.marks.length,
      units:'wind mph/deg, hold+miss mil (+R/+high), distance in, DA ft'},
    stats:computeStats(logState.marks),
    marks:logState.marks, samples:logState.samples
  },null,1);
}
function buildSummaryCsv(){
  var head=['iso','t_rel_s','event','shot','n_nodes','wtd_spd_mph','wtd_dir_deg','wtd_reldir_deg',
    'cross_mph_R','head_mph','gust_mph','hold_wind_mil','hold_wind_moa','hold_elev_mil','hold_elev_moa',
    'dialed_wind_mil','dialed_elev_mil','tof_s','vrem_fps','sg','da_ft','stab_tier','cur_spdSd','cur_dirSd',
    'w60_spdSd','w60_dirSd','sent_agree_frac','base_tempF','base_presInHg','conv','missH_in','missV_in','note'];
  var rows=[head];
  var evs=[];
  logState.samples.forEach(function(s){evs.push({t:s.t,kind:'sample',o:s});});
  logState.marks.forEach(function(m){evs.push({t:m.t,kind:'mark',o:m});});
  evs.sort(function(a,b){return a.t-b.t;});
  evs.forEach(function(e){
    var s=e.o, hold=s.hold||{}, w=s.wtd||{}, stab=s.stab||{}, sent=s.sent||{}, base=s.base||{};
    rows.push([isoT(s.t), (s.tRel!=null?s.tRel.toFixed(2):''), e.kind, (e.kind==='mark'?s.shot:''),
      s.nCount, num(w.spd,2), num(w.dir,0), num(w.relDir,0), num(w.cross,2), num(w.head,2), num(w.gust,2),
      num(hold.windMil,2), num(hold.windMOA,2), num(hold.elevMil,2), num(hold.elevMOA,2),
      num(hold.dialedWindMil,2), num(hold.dialedElevMil,2), num(hold.tof,2), num(hold.vRemain,0), num(hold.sg,2),
      num(s.da,0), (stab.tier||''), num(stab.curSpdSd,2), num(stab.curDirSd,0), num(stab.w60SpdSd,2), num(stab.w60DirSd,0),
      (sent&&sent.frac!=null?num(sent.frac,2):''), num(base.tempF,1), num(base.presInHg,2),
      (e.kind==='mark'?s.conv:''), (e.kind==='mark'?num(s.missH_in,2):''), (e.kind==='mark'?num(s.missV_in,2):''),
      (e.kind==='mark'?(s.note||''):'')]);
  });
  return rows.map(csvRow).join('\n');
}
function buildNodesCsv(){
  var head=['iso','t_rel_s','node_id','spd_mph','dir_deg','gust_mph','spd_sd','dir_sd','tempF','presInHg','battV','rssi','age_s','lat','lon'];
  var rows=[head];
  logState.samples.forEach(function(s){(s.nodes||[]).forEach(function(n){
    rows.push([isoT(s.t),(s.tRel!=null?s.tRel.toFixed(2):''),n.id,num(n.spd,2),num(n.dir,0),num(n.gust,2),
      num(n.spdSd,2),num(n.dirSd,1),num(n.tempF,1),num(n.presInHg,2),num(n.battV,2),num(n.rssi,0),n.age,
      num(n.lat,6),num(n.lon,6)]);
  });});
  return rows.map(csvRow).join('\n');
}
function num(v,d){return (v===null||v===undefined||isNaN(v))?'':(+v).toFixed(d);}

function tstamp(){return isoT(Date.now()).replace(/[:.]/g,'-').slice(0,19);}
function exportJson(){downloadFile('windlog_'+tstamp()+'.json','application/json',buildSessionJson());}
function exportSummaryCsv(){downloadFile('windlog_'+tstamp()+'_summary.csv','text/csv',buildSummaryCsv());}
function exportNodesCsv(){downloadFile('windlog_'+tstamp()+'_nodes.csv','text/csv',buildNodesCsv());}

function downloadFile(name,mime,text){
  if(!text||(!logState.samples.length&&!logState.marks.length)){flash('nothing logged yet');return;}
  try{
    var blob=new Blob([text],{type:mime});
    var url=URL.createObjectURL(blob);
    var a=document.createElement('a');a.href=url;a.download=name;
    document.body.appendChild(a);a.click();
    setTimeout(function(){try{document.body.removeChild(a);}catch(e){}URL.revokeObjectURL(url);},1500);
    flash('exported '+name);
  }catch(e){showRaw(name,text);}
}
// iOS Safari sometimes ignores the download attribute -- fallback shows the raw
// text in a selectable box so it can be copied out.
function showRaw(name,text){
  var host=document.getElementById('rawHost')||document.body;
  host.innerHTML='<div class="rawmodal" onclick="if(event.target===this)this.remove()">'+
    '<div style="width:100%;max-width:640px"><div class="statline" style="margin-bottom:6px">'+name+' &mdash; select all &amp; copy</div>'+
    '<textarea readonly onclick="this.select()"></textarea>'+
    '<div class="stepRow"><button class="step" onclick="this.closest(\'.rawmodal\').remove()">close</button></div></div></div>';
  host.querySelector('textarea').value=text;
}

// ---- fleet sleep (BASIC + FORECAST) ----------------------------------------
// One button, two meanings. The confirm() on SLEEP is deliberate: this takes
// the entire fleet quiet for up to 5 minutes at a stretch, mid-string, on one
// tap -- worth one extra tap to not do by accident. WAKE needs no confirm.
var sleepCmdBusy=false;
async function toggleFleetSleep(){
  if(sleepCmdBusy||!data||!data.sleep)return;
  var toSleep=!data.sleep.mode;
  if(toSleep&&!confirm('Put all nodes to sleep?\n\nThey will check in on their own schedule (20 s when warm and charged, up to 5 min in the cold) until you press WAKE.'))return;
  sleepCmdBusy=true;
  try{
    var r=await fetch('/cmd?op='+(toSleep?'sleep':'wake'));
    await r.json();
    poll();                       // refresh state immediately
  }catch(e){flash('command failed');}
  finally{sleepCmdBusy=false;}
}
async function rewakeFleet(){
  // Re-broadcasts the 30 s WAKE_CMD window. Use when a node missed the first
  // attempt -- it will catch the command on its next scheduled check-in.
  if(sleepCmdBusy)return;
  sleepCmdBusy=true;
  try{
    var r=await fetch('/cmd?op=wake');
    await r.json();
    poll();
  }catch(e){flash('re-wake failed');}
  finally{sleepCmdBusy=false;}
}
// ---- per-node sleep (v5) ----
// Layered ON TOP of the fleet button: a node can be individually parked even
// while the fleet is awake. Reflects/commands the receiver's nodeSleepMask via
// /cmd?op=sleep|wake&node=N. Distinct from the fleet bar, which stays on the
// BASIC/FORECAST tabs; this control lives inside each individual node's tab.
var nodeSleepBusy=false;
async function toggleNodeSleep(id){
  if(nodeSleepBusy||!data||!data.sleep)return;
  var mask=data.sleep.nodeMask||0;
  var isAsleep=((mask>>id)&1)!==0;
  var toSleep=!isAsleep;
  if(toSleep&&!confirm('Put node '+id+' to sleep?\n\nIt will drop out of the schedule and check in on its own until you wake it (or press WAKE on a fleet-wide wake).'))return;
  nodeSleepBusy=true;
  try{
    var r=await fetch('/cmd?op='+(toSleep?'sleep':'wake')+'&node='+id);
    var j=await r.json();
    if(j&&j.ok===false){flash('node '+id+' command rejected');}
    else{flash('node '+id+(toSleep?' sleeping':' waking'));}
    poll();                       // refresh state immediately
  }catch(e){flash('command failed');}
  finally{nodeSleepBusy=false;}
}
function renderNodeSleep(){
  var row=document.getElementById('nodeSleepRow');if(!row)return;
  // Only inside an individual node's tab (view is that node's numeric id).
  var isNodeTab=!(view==='avg'||view==='pred'||view==='solver'||view==='log');
  if(!isNodeTab||!data||!data.sleep){row.className='hide';return;}
  var id=parseInt(view,10);
  if(!(id>=1&&id<=15)){row.className='hide';return;}
  row.className='';
  var mask=data.sleep.nodeMask||0;
  var cmd=((mask>>id)&1)!==0;                         // commanded to sleep
  var nd=(data.nodes||[]).find(function(x){return x.id==id;});
  var acked=nd&&nd.sleeping;                          // has actually acked+gone quiet
  var fleet=data.sleep.mode;                          // fleet-wide sleep active
  var st=data.sleep;
  // Amber in both states, matching the whole-fleet sleep button. The SLEEP vs
  // WAKE distinction is carried by the label, not the color (same as the fleet bar).
  var btnLbl, sub, bg='var(--fire)';
  if(cmd){
    btnLbl='WAKE NODE '+id;
    sub=acked?('<b style="color:var(--ink)">NODE '+id+' SLEEPING</b> &middot; acked, checking in periodically')
             :('<b style="color:var(--ink)">NODE '+id+' SLEEP SET</b> &middot; waiting for it to ack on its next slot');
  }else{
    btnLbl='SLEEP NODE '+id;
    sub=fleet?('Node '+id+' is awake individually, but the FLEET is asleep &middot; fleet WAKE controls it')
             :('Node '+id+' is active and scheduled');
  }
  // Surface the fleet WAKE-broadcast countdown on the node tab too. It normally
  // lives only in the fleet sleep bar, which is hidden on individual node tabs --
  // so from here the countdown looked like it never started until you switched
  // to the Basic/Forecast tab. render() re-runs every poll, so this ticks down live.
  if(st.waking){
    sub+=' &middot; <b style="color:var(--fireInk)">WAKE BROADCAST '+st.wakeRemain+'s left</b>';
  }
  // If the fleet is asleep, the per-node control is informational: the fleet
  // command already sleeps everyone. Disable to avoid a confusing no-op.
  var dis=fleet&&!cmd;
  row.innerHTML=
    '<div style="display:flex;align-items:center;gap:12px;padding:12px 14px;border:1px solid var(--line,#333);border-radius:12px;background:var(--panel,#1a1a1a)">'+
      '<button onclick="toggleNodeSleep('+id+')" '+(dis?'disabled ':'')+
        'style="flex:0 0 auto;padding:12px 16px;border-radius:10px;border:1px solid '+bg+';background:'+bg+';color:var(--onFire);font-weight:700;'+(dis?'opacity:.45;':'')+'">'+btnLbl+'</button>'+
      '<div style="flex:1 1 auto;font-size:13px;color:var(--dim,#9aa)">'+sub+'</div>'+
    '</div>';
}
function renderSleepBar(){
  var bar=document.getElementById('sleepBar');if(!bar)return;
  // Only on the two top-level tabs of BASIC / FORECAST -- not on per-node
  // tabs, the solver, or LOG, where it would just be clutter.
  if(!data||!data.sleep||(view!=='avg'&&view!=='pred')){bar.className='hide';return;}
  bar.className='';bar.style.display='flex';
  var st=data.sleep,btn=document.getElementById('sleepBtn'),info=document.getElementById('sleepInfo');
  var rewake=document.getElementById('rewakeBtn');
  var nSlp=(data.nodes||[]).filter(function(n){return n.sleeping;}).length;
  var nTot=(data.nodes||[]).length;
  if(st.mode){
    btn.textContent='WAKE NODES';
    btn.style.background='var(--fire)';btn.style.color='var(--onFire)';btn.style.borderColor='var(--fire)';
    info.innerHTML='<b style="color:var(--ink)">FLEET SLEEPING</b> &middot; '+nSlp+'/'+nTot+
      ' acked &middot; nodes check in every 20 s&ndash;5 min (battery/temp)';
    rewake.className='hide';
  }else if(st.waking){
    btn.textContent='WAKING…';
    btn.style.background='var(--fire)';btn.style.color='var(--onFire)';btn.style.borderColor='var(--fire)';
    info.innerHTML='<b style="color:var(--ink)">WAKE BROADCAST</b> &middot; '+st.wakeRemain+
      ' s left &middot; '+nSlp+' node'+(nSlp==1?'':'s')+' still down (caught as they check in)';
    rewake.className='hide';
  }else{
    btn.textContent='SLEEP NODES';
    btn.style.background='var(--fire)';btn.style.color='var(--onFire)';btn.style.borderColor='var(--fire)';
    info.innerHTML=nSlp
      ?('<b style="color:var(--warn)">'+nSlp+' node'+(nSlp==1?'':'s')+' still sleeping</b> &middot; press RE-WAKE or wait for next check-in')
      :'All nodes awake &middot; sleep saves battery between strings';
    // RE-WAKE appears only when nodes are still sleeping after the 30 s wake
    // window expired -- lets you re-broadcast without having to sleep first.
    if(nSlp>0){rewake.className='';rewake.style.display='';rewake.textContent='RE-WAKE';}
    else{rewake.className='hide';}
  }
}

function setSection(s){
  // Cancel any pending sensitivity-fetch debounce (user may have typed in the
  // range field then switched tabs before the 600 ms timer fired), plus any
  // in-flight "computing" re-poll -- neither should keep firing off-tab.
  if(sbSensDebounceTimer){ clearTimeout(sbSensDebounceTimer); sbSensDebounceTimer=null; }
  if(sbSensPollTimer){ clearTimeout(sbSensPollTimer); sbSensPollTimer=null; }
  // Resume the live-data poll if Sandbox had suspended it
  if(!pollTimer){ poll(); pollTimer=setInterval(poll,350); }
  section=s;localStorage.setItem('section',s);
  document.getElementById('secBasic').className='sec'+(s==='basic'?' on':'');
  document.getElementById('secFore').className='sec'+(s==='forecast'?' on':'');
  document.getElementById('secLog').className='sec'+(s==='log'?' on':'');
  var _ss=document.getElementById('secSand'); if(_ss)_ss.className='sec'+(s==='sandbox'?' on':'');
  var isLog=(s==='log');
  document.getElementById('tabs').className=isLog?'tabs hide':'tabs';
  document.getElementById('wheelBlock').className=isLog?'hide':'';
  document.getElementById('logView').className=isLog?'':'hide';
  if(isLog){
    view='log';
    // Hide the sandbox view too. This branch returns before the shared
    // sandboxView-hide below, so without this a Sandbox->Log switch left the
    // sandbox panel covering the page (Log looked "stuck" until you went via
    // Basic/Forecast, which do reach that hide). This was the nav bug.
    document.getElementById('sandboxView').className='hide';
    document.getElementById('windView').className='hide';
    document.getElementById('solveView').className='hide';
    document.getElementById('stabView').className='hide';
    renderLog();
    return;
  }
  var isSand=(s==='sandbox');
  document.getElementById('secSand').className='sec'+(isSand?' on':'');
  document.getElementById('sandboxView').className=isSand?'':'hide';
  if(isSand){
    document.getElementById('tabs').className='tabs hide';
    document.getElementById('wheelBlock').className='hide';
    document.getElementById('windView').className='hide';
    document.getElementById('solveView').className='hide';
    document.getElementById('stabView').className='hide';
    document.getElementById('logView').className='hide';
    sbShow();
    return;
  }
  document.getElementById('sandboxView').className='hide';
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
// The SOLVER tab opens the Skyward-style solver (sky.js) full screen. The
// classic one-page form is still here: More -> Classic Coriolis solver, or if
// sky.js failed to load.
var skyClassic=false;
function openClassicSolver(){skyClassic=true;try{setView('solver');}finally{skyClassic=false;}}
function setView(v){
  if(v==='solver'&&!skyClassic&&window.SkyUI&&SkyUI.open()){tabs();return;}
  view=v;
  var isSolver=(v==='solver'), isPred=(v==='pred');
  document.getElementById('windView').className=isSolver?'hide':'';
  document.getElementById('solveView').className=isSolver?'':'hide';
  document.getElementById('stabView').className=isPred?'':'hide';
  if(isSolver){renderSolver();if(window.Gear)Gear.sync();}
  render();
}

function tele(){
  var g=document.getElementById('tele');
  function c(k,v){return '<div class="card"><div class="k">'+k+'</div><div class="v">'+v+'</div></div>';}
  if(view==='avg'||view==='pred'){var a=data.avg;var ww=weightedAvgWind();
    // A dead meter must never be silent -- it's the difference between "3 nodes
    // agree it's calm" and "2 nodes plus a corpse reporting zero".
    var nodeLbl=ww.bad?('<span style="color:var(--warn)">'+ww.count+' (+'+ww.bad+' no wind)</span>'):a.count;
    // Meters that point different ways partly cancel in the vector average.
    // Say so, so a small net call isn't mistaken for calm, steady air.
    var agreeCard=(ww.count>1&&ww.agree<0.5)?'<div class="card"><div class="k">Meters</div><div class="v" style="color:var(--warn)">DISAGREE '+Math.round(ww.agree*100)+'%</div></div>':'';
    g.innerHTML=agreeCard+c('Nodes',nodeLbl)+c('Wtd wind',ww.speed.toFixed(1)+' mph '+Math.round(ww.dir)+'&deg; '+card(ww.dir))+
      c('Avg temp',a.tempF.toFixed(0)+'&deg;F')+c('Avg baro',a.presInHg.toFixed(2)+' inHg');
  }else{var n=(data.nodes||[]).find(function(x){return x.id==view;});if(!n){g.innerHTML='';return;}
    var lux=(n.lux<0)?'&mdash;':n.lux.toFixed(0)+' lx',dist=(n.distYd<0)?'&mdash;':n.distYd+' yd';
    var ml=getMult(n.id);
    var mcard=(ml!==1)?c('Wind mult','&times;'+ml.toFixed(2)+' = '+(n.speed*ml).toFixed(1)+' mph'):'';
    var mt=getNodeMeterType(n.id);
    var vaneLbl=(mt>=0)?METER_NAMES[mt]:'not set';
    // windOk:false = this node is not voting on the wind call. windStale says
    // the meter is fine but the reading is too old (a sleeping node between
    // check-ins), which is a different thing from a dead meter.
    var wokcard=(n.windOk===false)?'<div class="card"><div class="k">Wind sensor</div><div class="v" style="color:var(--warn)">'+(n.windStale?'OLD READING':'NO DATA')+'</div></div>':'';
    g.innerHTML=wokcard+c('True dir',Math.round(n.dir)+'&deg; '+card(n.dir))+c('Temp',n.tempF.toFixed(0)+'&deg;F')+
      c('Baro',n.presInHg.toFixed(2)+' inHg')+c('Light',lux)+
      c('Wind meter',vaneLbl)+
      c('Battery',n.battV.toFixed(2)+'V &middot; '+n.battPct+'%')+c('Distance',dist)+
      c('Signal',n.rssi+' dBm')+c('Updated',n.age+'s ago')+mcard;
  }
}

function render(){
  renderSleepBar();
  renderNodeSleep();
  renderAimRow();
  if(!data)return;
  if(view==='log')return;
  tabs();
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
  if(view==='pred'){renderStability();updateWindHold();renderLogBar();}
  // BASIC (avg) also needs a live hold so the Aimbot button has a current value
  // to dial. The windHold div isn't shown here, but updateWindHold still sets
  // aimHoldMOA; the throttle + in-flight guard keep this from adding load.
  else if(view==='avg'&&data&&data.aim&&data.aim.present){updateWindHold();}
  var b=data.base;
  document.getElementById('base').innerHTML='Base station &middot; '+
    (b.gps?(b.lat.toFixed(5)+', '+b.lon.toFixed(5)):'no GPS fix')+'<br>'+
    b.tempF.toFixed(0)+'&deg;F &middot; '+b.presInHg.toFixed(2)+' inHg &middot; battery '+b.battPct+'%';
}

// ---- solver UI ----
function setCurve(el){
  var t=el.value.trim();
  if(t&&window.Gear){try{Gear.parseCurve(t);}catch(e){flash(e.message);el.style.borderColor='var(--warn)';return;}}
  el.style.borderColor='';setv('dragCurve',t);
  if(t&&getv('model',7)!=='C'){setv('model','C');renderSolver();flash('Drag model set to Custom curve');}
}
function renderSolver(){
  var h='';
  LOAD.forEach(function(f){
    if(f.opt){var opts=f.opt.slice();
      if(f.k==='model')opts.push(['C','Custom curve']);
      var o='';opts.forEach(function(p){o+='<option value="'+p[0]+'"'+((''+getv(f.k,f.d))==(''+p[0])?' selected':'')+'>'+p[1]+'</option>';});
      h+='<div class="field"><span class="lab">'+f.l+'</span><select onchange="setv(\''+f.k+'\',this.value)">'+o+'</select></div>';
    }else if(f.t==='area'){
      h+='<div class="field" style="flex-wrap:wrap"><span class="lab">'+f.l+'</span><textarea rows="3" placeholder="'+(f.ph||'')+'" style="width:100%;margin-top:6px;padding:8px;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;font-family:var(--mono);font-size:.85rem" onchange="setCurve(this)">'+escHtml(getv(f.k,f.d))+'</textarea></div>';
    }else if(f.t==='text'){
      h+='<div class="field"><span class="lab">'+f.l+'</span><input type="text" placeholder="'+(f.ph||'')+'" value="'+String(getv(f.k,f.d)).replace(/"/g,'&quot;')+'" onchange="setv(\''+f.k+'\',this.value.trim())"></div>';
    }else{
      h+='<div class="field"><span class="lab">'+f.l+'</span><input type="number" step="any" value="'+getv(f.k,f.d)+'" onchange="setv(\''+f.k+'\',this.value)"></div>';
    }
  });
  h+='<div class="field"><span class="lab" style="color:var(--good)">Spin drift</span><input class="lm" type="checkbox" style="width:24px;height:24px" id="chkSpin" '+((localStorage.getItem('opt_spin')||'1')==='1'?'checked':'')+' onchange="localStorage.setItem(\'opt_spin\',this.checked?1:0)"></div>';
  h+='<div class="field"><span class="lab" style="color:var(--good)">Aero jump</span><input class="lm" type="checkbox" style="width:24px;height:24px" id="chkJump" '+((localStorage.getItem('opt_jump')||'1')==='1'?'checked':'')+' onchange="localStorage.setItem(\'opt_jump\',this.checked?1:0)"></div>';
  h+='<div class="field"><span class="lab" style="color:var(--good)">Earth (Coriolis/Eotvos)</span><input class="lm" type="checkbox" style="width:24px;height:24px" '+((localStorage.getItem('opt_earth')||'0')==='1'?'checked':'')+' onchange="localStorage.setItem(\'opt_earth\',this.checked?1:0)"></div>';
  var sm=localStorage.getItem('opt_spinModel')||'LITZ_CLASSIC';
  h+='<div class="field"><span class="lab">Spin drift formula</span><select onchange="localStorage.setItem(\'opt_spinModel\',this.value)">'+
     [['LITZ_CLASSIC','Litz (Coriolis)'],['LITZ','Litz, zeroed (Skyward)'],['MEASURED','Measured (Skyward)']].map(function(o){return '<option value="'+o[0]+'"'+(o[0]===sm?' selected':'')+'>'+o[1]+'</option>';}).join('')+'</select></div>';
  LIVE.forEach(function(f){
    var on=liveOn(f.live);
    h+='<div class="field"><span class="lab">'+f.l+'</span>'+
       '<input id="in_'+f.k+'" type="number" step="any" value="'+getv(f.k,f.d)+'" '+(on?'disabled':'')+' onchange="setv(\''+f.k+'\',this.value)">'+
       '<button class="lm'+(on?' live':'')+'" onclick="toggleLive(\''+f.live+'\')">'+(on?'LIVE':'MAN')+'</button></div>';
  });
  document.getElementById('loadFields').innerHTML=h;
  renderGearLists();
  syncLiveInputs();
}
function escHtml(t){return String(t).replace(/[&<>"]/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c];});}
// Profile dropdown + bullet picker + gear status (cheap; safe while typing)
function renderGearLists(){
  var sel=document.getElementById('profileSel');
  if(sel&&window.Gear){var act=activeLoadId();
    var o='<option value="">-- select / unsaved --</option>';
    Gear.loads().forEach(function(l){o+='<option value="'+escHtml(l.id)+'"'+(l.id===act?' selected':'')+'>'+escHtml(Gear.label(l))+'</option>';});
    sel.innerHTML=o;}
  var bs=document.getElementById('bulletSel');
  if(bs&&window.Gear){
    var cal=parseFloat(getv('cal',0.308))||0,all=document.getElementById('bulletAll')&&document.getElementById('bulletAll').checked;
    var cur=getv('bulletId','');
    var list=Gear.bullets().filter(function(b){return all||Math.abs(b.diameterIn-cal)<=0.0015||b.id===cur;})
      .sort(function(a,b){var x=((a.maker||'')+' '+a.name).toLowerCase(),y=((b.maker||'')+' '+b.name).toLowerCase();return x<y?-1:x>y?1:0;});
    var o2='<option value="">'+(list.length?'-- pick a bullet ('+list.length+(all?'':' for this caliber')+') --':'-- no bullets'+(all?'':' for this caliber')+' --')+'</option>';
    var labs=list.map(function(b){
      var rows=b.bcPoints||[],tag=b.dragModel==='CUSTOM'?'custom curve':(b.dragModel+' '+(rows.length?rows.slice().sort(function(x,y){return y.minVelFps-x.minVelFps;})[0].bc:'?')+(rows.length>1?' +'+(rows.length-1)+' bands':''));
      // names made up by this page already say weight/model/BC
      if(/^[0-9.]+ gr (G1|G7|CUSTOM)/.test(b.name||''))return b.name+(rows.length>1?' +'+(rows.length-1)+' bands':'')+(b.dragModel==='CUSTOM'?' curve':'');
      return (b.maker?b.maker+' ':'')+b.name+' \u00b7 '+b.weightGr+' gr \u00b7 '+tag;});
    var seen={};labs.forEach(function(t){seen[t]=(seen[t]||0)+1;});
    var k={};list.forEach(function(b,i){var t=labs[i];
      if(seen[t]>1){k[t]=(k[t]||0)+1;t+=' ('+k[t]+')';}
      o2+='<option value="'+escHtml(b.id)+'"'+(b.id===cur?' selected':'')+'>'+escHtml(t)+'</option>';});
    bs.innerHTML=o2;
  }
  renderGearStatus();
}
function syncLiveInputs(){
  if(!data)return;
  if(liveOn('live_temp')){var e=document.getElementById('in_tempF');if(e)e.value=data.base.tempF.toFixed(1);}
  if(liveOn('live_pres')){var e=document.getElementById('in_pres');if(e)e.value=data.base.presInHg.toFixed(2);}
  if(liveOn('live_wind')&&avgWind().ok){var w=avgWind();
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
  p.az=fireAz;p.bulletId=getv('bulletId','');p.loadId=activeLoadId();
  p.spinModel=localStorage.getItem('opt_spinModel')||'LITZ_CLASSIC';
  // Shot inputs the Skyward-style Solver screen adds (all default to "off"),
  // so WIND HOLD, the log and the aimbot dial exactly what that screen shows.
  p.look=getv('look',0);p.cant=getv('cant',0);p.tgtMph=getv('tgtMph',0);p.wind2=getv('wind2',0);
  p.hum=getv('hum',0);p.alt=getv('alt',0);p.localG=localStorage.getItem('opt_localG')||'0';
  p.presAbs=liveOn('live_pres')?'1':getv('presAbs','1');
  return p;
}
// Ballistics run on THIS PHONE (engine.js, a port of the Skyward engine). Only
// if that script failed to load does the page ask the receiver (/solve) the
// old way -- which blocks the base station's radio loop while it computes.
function engineLocal(){return !!(window.Gear&&window.SkyEngine);}
async function solveP(p){
  if(engineLocal())return Gear.solve(p);
  var q='/solve?mv='+p.mv+'&bc='+p.bc+'&model='+(p.model==='C'?7:p.model)+'&wgt='+p.wgt+'&cal='+p.cal+
    '&twist='+p.twist+'&blen='+p.blen+'&twistDir='+p.twistDir+'&sh='+p.sh+'&zero='+p.zero+
    '&range='+p.range+'&tempF='+p.tempF+'&pres='+p.pres+'&windmph='+p.windmph+'&windrel='+p.windrel+
    '&lat='+p.lat+'&az='+fireAz+'&earth='+p.earth+'&spin='+p.spin+'&jump='+p.jump;
  var r=await fetch(q,{cache:'no-store'});return await r.json();
}
function renderTable(){
  var box=document.getElementById('tableOut');if(!box)return;
  if(!engineLocal()){box.textContent='Table needs the phone solver (reload the page).';return;}
  var p;try{p=solverParams();}catch(e){box.textContent='Waiting for base station data.';return;}
  var step=Math.max(10,parseFloat((document.getElementById('tblStep')||{}).value)||100);
  var max=Math.max(step,parseFloat(p.range)||500);
  var g;try{g=Gear.engineGear(p);}catch(e){box.textContent=e.message;return;}
  var sol;try{sol=SkyEngine.solve(g.rifle,g.load,g.bullet,{distanceYd:max,windSpeed1Mph:+p.windmph||0,windAngleDeg:+p.windrel||90,
    spinDrift:p.spin!=='0',aeroJump:p.jump!=='0',spinDriftModel:p.spinModel,coriolis:p.earth==='1',latitudeDeg:+p.lat||0,azimuthDeg:+fireAz||0,
    atmosphere:{temperatureF:+p.tempF,pressureInHg:+p.pres,pressureIsAbsolute:true}},{maxRangeYd:max,stepYd:step,extraRangesYd:[]});}
  catch(e){box.textContent='Solver: '+e.message;return;}
  var emag=parseFloat(getv('zeroElevMag',0))||0,edir=getv('zeroElevDir','high'),wmag=parseFloat(getv('zeroWindMag',0))||0,wdir=getv('zeroWindDir','right');
  var ze=(edir==='high')?-emag:emag,zw=(wdir==='right')?-wmag:wmag,K=SkyEngine.MOA_PER_MIL;
  var h='<table style="width:100%;border-collapse:collapse"><tr style="color:var(--dim)"><td>yd</td><td>elev MOA</td><td>mil</td><td>wind MOA</td><td>mil</td><td>fps</td><td>TOF</td></tr>';
  sol.rows.forEach(function(r){if(r.rangeYd<=0)return;
    var e=r.comeUpMil*K+ze,w=r.windageMil*K+zw;
    h+='<tr><td>'+Math.round(r.rangeYd)+'</td><td>'+e.toFixed(1)+'</td><td>'+(e/K).toFixed(2)+'</td><td>'+Math.abs(w).toFixed(1)+(w>=0?'R':'L')+
       '</td><td>'+Math.abs(w/K).toFixed(2)+(w>=0?'R':'L')+'</td><td>'+Math.round(r.velocityFps)+'</td><td>'+r.tofS.toFixed(2)+'</td></tr>';});
  h+='</table><div style="color:var(--dim);margin-top:6px">'+
     (sol.transonicYd?'Transonic (Mach 1.2) at '+Math.round(sol.transonicYd)+' yd. ':'')+
     (sol.subsonicYd?'Subsonic at '+Math.round(sol.subsonicYd)+' yd. ':'')+'Wind as entered above, group offsets applied.</div>';
  box.innerHTML=h;
}
async function doSolve(){
  if(!data)return;
  if(solveBusy)return;          // share the single-in-flight guard with updateWindHold
  solveBusy=true;
  try{ await doSolveInner(); } finally { solveBusy=false; autoSolveLast=Date.now(); }
}
async function doSolveInner(){
  if(liveWindMissing()){document.getElementById('results').innerHTML='<div class="card" style="color:var(--warn)">'+(liveStale()?'Lost the base station: no live wind. Rejoin its Wi-Fi, or turn off live wind and enter it by hand.':'No meter has valid wind. Turn off live wind and enter it by hand, or check the meters.')+'</div>';return;}
  var p=solverParams();
  try{
    var s=await solveP(p);
    var g=document.getElementById('results');
    function c(k,v){return '<div class="card"><div class="k">'+k+'</div><div class="big">'+v+'</div></div>';}
    if(!s.ok){g.innerHTML='<div class="card">Solver could not converge - check inputs.'+(s.err&&s.err!=='range'?' ('+escHtml(s.err)+')':'')+'</div>';return;}
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
    var tb=document.getElementById('tableBox');if(tb&&tb.open)renderTable();
  }catch(e){
    document.getElementById('results').innerHTML='<div class="card">Solve failed.</div>';
  }
}
var autoSolveLast=0;
function maybeAutoSolve(){
  if(view!=='solver')return;
  if(!(liveOn('live_temp')||liveOn('live_pres')||liveOn('live_wind')))return;
  // Same throttle as WIND HOLD: live inputs jitter every poll, so this fired a
  // full receiver-blocking solve every 350 ms while the Solver tab was open.
  if(solveBusy)return;
  if(Date.now()-autoSolveLast<minMs(AUTOSOLVE_MIN_MS))return;
  autoSolveLast=Date.now();
  doSolve();
}

async function poll(){
  try{
    var r=await fetch('/data.json',{cache:'no-store'});data=await r.json();dataAt=Date.now();
    var s=document.getElementById('status');s.textContent='live';s.className='status live';
    if(view!=='avg'&&view!=='pred'&&view!=='solver'&&view!=='log'&&!(data.nodes||[]).some(function(n){return n.id==view;}))view=(section==='forecast')?'pred':'avg';
    render();maybeAutoSolve();
    if(window.SkyUI&&SkyUI.isOpen())SkyUI.onData();
  }catch(e){var s=document.getElementById('status');s.textContent='offline';s.className='status off';
    // once the last reading is too old, every wind call must say there is none
    if(liveStale()){try{render();if(typeof updateWindHold==='function')updateWindHold(true);}catch(x){}
      if(window.SkyUI&&SkyUI.isOpen())SkyUI.onData();}}
}
// ===== SANDBOX MODULE =====
// ============================ SANDBOX (virtual BC lab) =======================
// Self-contained. Reuses the solver's gear store (window.Gear)
// and the same segmented wind model the device uses. No receiver round-trips.
var sbSub='setup';
var SB_N=15;
var sbNodes=[];        // {id,on,down,speed,rel,spdSd,dirSd}
var sbCoef=[];         // sensitivity per segment (inches/mph) on the DISPLAY grid
var sbNSeg=0, sbSegYd=25, sbRangeYd=500;
var sbLastSim=null;
// Real-curve cache from /sensitivity. sbRealCoef holds the device's per-segment
// coefficients on ITS grid (segYd may differ from the display grid); we sample
// it by downrange yard, so the display segment width is pure resolution.
var sbRealCoef=null, sbRealSegYd=0, sbRealRange=0, sbCoefSrc='approx';
var sbSensKey='';      // inputs the cached curve was fetched for (refetch on change)
var sbSensBusy=false;
// The receiver computes the curve on its other core now and answers
// {computing:true} until it's ready. We keep the approximate curve on screen and
// re-poll. Long ranges are genuinely slow on the device (~1s/500yd grows to ~60s
// near 1500yd), so the ceiling is generous; the box stays responsive throughout.
var sbSensPollTimer=null, sbSensPollN=0, sbPopulateT0=0;
var SB_POLL_MS=1000;          // interval between "still computing?" polls
var SB_POLL_MAX=150;          // ceiling (~150 s) -- covers up to ~1800 yd

function sbInit(){
  if(sbNodes.length) return;
  for(var i=1;i<=SB_N;i++) sbNodes.push({id:i,on:i<=5,down:-1,speed:10,rel:90,spdSd:1.5,dirSd:8});
  sbAutoDistribute();
}

// ---- "Populate" button: explicit, gated true-curve pull --------------------
// The device curve takes 2-4 s to compute, so we don't pull it automatically on
// tab entry / profile load anymore -- the chart shows the fast approximate shape
// immediately and the user asks for the true curve deliberately with Populate.
// Gate: a SAVED solver profile must be selected AND carry a real ballistic load
// (mv/bc/cal/wgt) -- the inputs /sensitivity needs. Without that the receiver
// can't answer, so the button stays disabled.
// Gate: the original ask was just "must select a saved profile" -- the stricter
// per-field mv>0&&bc>0&&cal>0&&wgt>0 check went beyond that AND, combined with the
// snapshot() bug above, could reject a genuinely-selected profile with no visible
// reason why. sbSensParams() (what actually builds the /sensitivity request)
// already substitutes a sensible default for any missing field, same as the live
// solver does -- so requiring a NAME is enough; the request behind it is already
// robust to gaps, including in profiles saved before the snapshot() fix.
function sbHasProfile(){
  var name=(document.getElementById('sbProfileSel')||{}).value||'';
  return name!=='';
}
// Reflect state on the button + hint. computing=true -> show progress + lock it.
function sbSyncPopulateBtn(computing){
  var btn=document.getElementById('sbPopulateBtn');
  var hint=document.getElementById('sbPopulateHint');
  var has=sbHasProfile();
  if(btn){
    if(computing){ btn.disabled=true; btn.textContent='Populating\u2026'; }
    else { btn.textContent=(sbCoefSrc==='device')?'Repopulate':'Populate'; btn.disabled=!has; }
  }
  if(hint) hint.style.display=has?'none':'';
}
// Drop any loaded device curve back to the approximate shape and re-arm the
// button. Called when an input that changes the curve (range, profile) changes,
// so we never show a stale device curve labelled as true.
function sbInvalidateCurve(){
  if(sbSensPollTimer){clearTimeout(sbSensPollTimer);sbSensPollTimer=null;}
  sbSensPollN=0; sbPopulateT0=0; sbSensKey=''; sbRealCoef=null; sbCoefSrc='approx';
  sbSyncPopulateBtn(false);
  sbRecalc();
}
// The button action: fire the true-curve pull now (skip the keystroke debounce)
// and drive the poll to completion. Forces a fresh pull even if a curve was
// cached for these inputs.
function sbPopulateCurve(){
  if(!sbHasProfile()){flash('Select a saved rifle profile first');return;}
  if(sbSensDebounceTimer){clearTimeout(sbSensDebounceTimer);sbSensDebounceTimer=null;}
  if(sbSensPollTimer){clearTimeout(sbSensPollTimer);sbSensPollTimer=null;}
  sbSensPollN=0; sbSensKey=''; sbRealCoef=null; sbPopulateT0=Date.now();
  sbSyncPopulateBtn(true);
  var src=document.getElementById('sbCurveSrc');
  if(src)src.innerHTML='<span style="color:var(--dim)">computing true curve\u2026 0s</span>';
  sbFetchSensitivity();
}

// Key of the ballistic inputs that change the sensitivity curve. Wind does NOT
// affect the curve (same reasoning as Sensitivity.h), so it's excluded -- the
// curve is cached across wind edits and only refetched when geometry/load change.
function sbProfileFlat(id){
  if(!id||!window.Gear||!Gear.load(id))return {};
  var f=Gear.flat(id);f.loadId=id;return f;
}
function sbSensParams(){
  var id=(document.getElementById('sbProfileSel')||{}).value||'';
  var p=sbProfileFlat(id);
  function g(k,d){return (p[k]!==undefined&&p[k]!=='')?p[k]:d;}
  return {loadId:id, flat:p,
    mv:g('mv',2700), bc:g('bc',0.243), model:g('model',7), wgt:g('wgt',175),
    cal:g('cal',0.308), twist:g('twist',10), blen:g('blen',1.24), sh:g('sh',1.75),
    zero:g('zero',100),
    range:Math.max(25,parseFloat((document.getElementById('sbRange')||{}).value)||500),
    tempF:g('tempF',59), pres:g('pres',29.92)
  };
}

// Fetch the TRUE per-segment curve from the receiver, cached. Falls back to the
// offline power-law shape when there's no receiver or no profile. Only fires on
// input change (keyed), never per Monte Carlo run.
async function sbFetchSensitivity(){
  var p=sbSensParams();
  var key=[p.loadId,JSON.stringify(p.flat),p.range,p.tempF,p.pres].join('|');
  if(key===sbSensKey && sbRealCoef) return;          // cache hit
  if(sbSensBusy) return;
  // Need a real load to ask the solver: MV/BC/cal/wgt must be present.
  if(!(p.mv>0&&p.bc>0&&p.cal>0&&p.wgt>0)){sbCoefSrc='approx';return;}
  if(engineLocal()&&p.loadId){
    var fp=Object.assign({},p.flat,{range:p.range,tempF:p.tempF,pres:p.pres});
    var c=Gear.sensitivity(fp,p.range);
    if(c.ok&&c.coef&&c.coef.length){
      sbRealCoef=c.coef.slice();sbRealSegYd=c.segYd;sbRealRange=c.rangeYd;sbCoefSrc='device';sbSensKey=key;
    }else{sbCoefSrc='approx';flash('sensitivity: '+(c.err||'could not compute curve for this profile'));}
    sbSensPollN=0;sbPopulateT0=0;sbSyncPopulateBtn(false);sbRecalc();return;
  }
  sbSensBusy=true;
  try{
    var q='/sensitivity?mv='+p.mv+'&bc='+p.bc+'&model='+p.model+'&wgt='+p.wgt+'&cal='+p.cal+
      '&twist='+p.twist+'&blen='+p.blen+'&sh='+p.sh+'&zero='+p.zero+'&range='+p.range+
      '&tempF='+p.tempF+'&pres='+p.pres+'&az='+(typeof fireAz!=='undefined'?fireAz:0);
    var r=await fetch(q,{cache:'no-store'});var s=await r.json();
    if(s.ok&&s.coef&&s.coef.length){
      sbRealCoef=s.coef.slice(); sbRealSegYd=s.segYd||(s.rangeYd/s.coef.length);
      sbRealRange=s.rangeYd||(sbRealSegYd*s.coef.length); sbCoefSrc='device';
      sbSensKey=key; sbSensPollN=0; sbPopulateT0=0; sbSyncPopulateBtn(false);
    }else if(s.computing){
      // Receiver is computing this curve on its other core. Keep the approximate
      // shape visible, show elapsed seconds so the user sees it's working (not
      // frozen), and poll again. If we exceed the ceiling, STOP and reset the
      // button -- never leave it stuck grey on "Populating".
      sbCoefSrc='approx';
      if(!sbPopulateT0) sbPopulateT0=Date.now();
      var secs=Math.round((Date.now()-sbPopulateT0)/1000);
      var src=document.getElementById('sbCurveSrc');
      if(sbSensPollN++<SB_POLL_MAX){
        sbSyncPopulateBtn(true);
        if(src)src.innerHTML='<span style="color:var(--dim)">computing true curve\u2026 '+secs+'s</span>';
        if(sbSensPollTimer){clearTimeout(sbSensPollTimer);sbSensPollTimer=null;}
        sbSensPollTimer=setTimeout(function(){sbSensPollTimer=null;sbFetchSensitivity();},SB_POLL_MS);
      }else{
        sbSensPollN=0; sbPopulateT0=0; sbSyncPopulateBtn(false);
        if(src)src.innerHTML='<span style="color:var(--warnY)">timed out after '+secs+'s \u2014 this range is very heavy; try a shorter range or press Populate again</span>';
        flash('sensitivity timed out ('+secs+'s) \u2014 long ranges are slow to compute');
      }
    }else{ sbCoefSrc='approx'; sbSensPollN=0; sbPopulateT0=0; sbSyncPopulateBtn(false);
      if(!s.computing) flash('sensitivity: could not compute curve for this profile'); }
  }catch(e){ sbCoefSrc='approx'; sbSensPollN=0; sbPopulateT0=0; sbSyncPopulateBtn(false); }
  finally{ sbSensBusy=false; }
  sbRecalc();
}
// Debounced wrapper: waits 600 ms after the last keystroke before fetching.
// Without this, typing "1000" fires four requests -- one per digit. The
// sbSensBusy guard drops calls 2-4, but those are the ones with the number you
// actually typed. The server computes the curve for "1", returns 2-4 s later,
// and the curve is wrong with nothing left to re-trigger it. With a debounce,
// only the final value fires and the user gets the right curve.
var sbSensDebounceTimer=null;
function sbFetchSensitivityDebounced(){
  if(sbSensDebounceTimer) clearTimeout(sbSensDebounceTimer);
  var src=document.getElementById('sbCurveSrc');
  if(src) src.innerHTML='<span style="color:var(--dim)">waiting\u2026</span>';
  sbSensDebounceTimer=setTimeout(function(){sbSensDebounceTimer=null;sbFetchSensitivity();},600);
}

// Sample the sensitivity curve at a downrange yard, in inches/mph. Uses the real
// device curve when available (linear interp between its segment centers, same
// math as Sensitivity::sensitivityAt), else the offline power-law shape.
function sbCoefAtYd(yd){
  if(sbCoefSrc==='device'&&sbRealCoef&&sbRealSegYd>0){
    var pos=yd/sbRealSegYd-0.5, n=sbRealCoef.length;
    if(pos<=0)return sbRealCoef[0];
    if(pos>=n-1)return sbRealCoef[n-1];
    var i=pos|0, f=pos-i;
    return sbRealCoef[i]*(1-f)+sbRealCoef[i+1]*f;
  }
  // ---- offline fallback: power-law SHAPE only (approximate, clearly labeled) ----
  // Not physical -- a stand-in for "muzzle wind matters more" when no device
  // curve is available. Real weighting is convex in TIME and can bump upward
  // through transonic; this smooth decay cannot show that.
  var f=(sbRangeYd>1)?yd/sbRangeYd:0; if(f<0)f=0; if(f>1)f=1;
  return (Math.pow(1-f,1.7)+0.06)*0.9*(sbRangeYd/500);
}

// Build the DISPLAY-grid coefficient array. Each entry is the drift contribution
// of ONE segment = (sensitivity density at its center) x (segment width in the
// device's own segment units). Scaling by width keeps the summed drift invariant
// to the display segment width -- width is now pure resolution, both in WHICH
// node owns a segment (coverage zones, above) AND in the magnitude (here).
function sbBuildCoef(){
  sbSegYd=Math.max(4,parseFloat(document.getElementById('sbSegYd').value)||25);
  sbRangeYd=Math.max(25,parseFloat(document.getElementById('sbRange').value)||500);
  sbNSeg=Math.max(2,Math.round(sbRangeYd/sbSegYd));
  // The curve's coefficients are "inches/mph per DEVICE segment". Convert to a
  // per-yard density, then re-integrate onto the display grid by x sbSegYd.
  var densRef=(sbCoefSrc==='device'&&sbRealSegYd>0)?sbRealSegYd:sbSegYd;
  sbCoef=[];
  for(var i=0;i<sbNSeg;i++){ sbCoef.push(sbCoefAtYd((i+0.5)*sbSegYd)/densRef*sbSegYd); }
}
function sbAutoDistribute(){
  var on=sbNodes.filter(function(n){return n.on;});
  var r=Math.max(25,parseFloat((document.getElementById('sbRange')||{}).value)||500);
  on.forEach(function(n,k){n.down=Math.round((k+0.5)*r/on.length);});
  sbRenderNodes();sbRecalc();
}
function sbAllOn(v){sbNodes.forEach(function(n){n.on=v;});sbRenderNodes();sbRecalc();}

function sbRenderNodes(){
  var t=document.getElementById('sbNodeTbl');if(!t)return;
  var h='<tr style="color:var(--dim);text-align:left">'+
    '<th>On</th><th>#</th><th>Dist yd</th><th>Wind</th><th>Dir&deg;</th><th>Spd SD</th><th>Dir SD</th></tr>';
  sbNodes.forEach(function(n,i){
    var dim=n.on?'':'opacity:.4';
    h+='<tr style="border-top:1px solid var(--line);'+dim+'">'+
      '<td><input type="checkbox" '+(n.on?'checked':'')+' onchange="sbSet('+i+',\'on\',this.checked)"></td>'+
      '<td>'+n.id+'</td>'+
      '<td><input class="sbi" type="number" value="'+n.down+'" oninput="sbSet('+i+',\'down\',this.value)"></td>'+
      '<td><input class="sbi" type="number" value="'+n.speed+'" oninput="sbSet('+i+',\'speed\',this.value)"></td>'+
      '<td><input class="sbi" type="number" value="'+n.rel+'" oninput="sbSet('+i+',\'rel\',this.value)"></td>'+
      '<td><input class="sbi" type="number" value="'+n.spdSd+'" oninput="sbSet('+i+',\'spdSd\',this.value)"></td>'+
      '<td><input class="sbi" type="number" value="'+n.dirSd+'" oninput="sbSet('+i+',\'dirSd\',this.value)"></td>'+
    '</tr>';
  });
  t.innerHTML=h;
}
function sbSet(i,k,v){
  if(k==='on')sbNodes[i].on=v;
  else sbNodes[i][k]=parseFloat(v)||0;
  if(k==='on')sbRenderNodes();
  sbRecalc();
}

// --- core math (shared with sim_core logic) ---
function sbCross(sp,rel){return sp*Math.sin(rel*Math.PI/180);}
// Assign each segment to a node by COVERAGE ZONE, not nearest-to-midpoint. Zone
// boundaries sit halfway between adjacent (sorted) active nodes, so a segment's
// owner depends only on where it falls between nodes -- NOT on the segment grid.
// This is why segment width is now pure resolution: refining the grid splits a
// zone into more (identical-valued) sub-segments instead of reshuffling owners.
// (Old code picked min|down-mid| per segment, so changing width moved midpoints,
// moved owners, and swung the drift + hit%. That was an artifact, now removed.)
function sbSegCross(nodes){
  var seg=new Array(sbNSeg).fill(0);
  var act=nodes.filter(function(n){return n.on&&n.down>=0;})
               .sort(function(a,b){return a.down-b.down;});
  if(!act.length)return seg;
  // zone boundaries: midpoints between consecutive nodes
  var bnd=[];
  for(var k=0;k<act.length-1;k++)bnd.push((act[k].down+act[k+1].down)/2);
  for(var s=0;s<sbNSeg;s++){
    var mid=(s+0.5)*sbSegYd;
    var z=0;while(z<bnd.length&&mid>bnd[z])z++;   // first node whose zone covers mid
    seg[s]=sbCross(act[z].speed,act[z].rel);
  }
  return seg;
}
function sbDrift(seg){var t=0;for(var i=0;i<sbNSeg;i++)t+=sbCoef[i]*seg[i];return t;}
function sbMoa(inch){return sbRangeYd>0?inch/(1.047*sbRangeYd/100):0;}
function sbGauss(m,sd,rng){if(sd<=0)return m;var u1=rng(),u2=rng();return m+sd*Math.sqrt(-2*Math.log(u1+1e-12))*Math.cos(2*Math.PI*u2);}
function sbRng(a){return function(){a|=0;a=a+0x6D2B79F5|0;var t=Math.imul(a^a>>>15,1|a);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};}

function sbRecalc(){
  sbBuildCoef();
  var seg=sbSegCross(sbNodes);
  var inch=sbDrift(seg);
  var nAct=sbNodes.filter(function(n){return n.on&&n.down>=0;}).length;
  var el;
  if(el=document.getElementById('sbCallMoa'))el.textContent=(sbMoa(inch)>=0?'R ':'L ')+Math.abs(sbMoa(inch)).toFixed(2)+' MOA';
  if(el=document.getElementById('sbCallIn'))el.textContent=inch.toFixed(1)+' in';
  if(el=document.getElementById('sbActive'))el.textContent=nAct+' / '+SB_N;
  var eff=nAct?(seg.reduce(function(a,b){return a+b;},0)/sbNSeg):0;
  if(el=document.getElementById('sbEffWind'))el.textContent=eff.toFixed(1)+' mph cross';
  sbDrawSens();
  if(el=document.getElementById('sbCurveSrc'))
    el.innerHTML=(sbCoefSrc==='device')
      ?'<span style="color:var(--good)">true curve</span> &middot; per-segment sensitivity for the loaded rifle'
      :'<span style="color:var(--warnY)">approximate shape</span> &middot; no true curve yet (pick a profile and press Populate); smooth decay, not physical';
  if(sbSub==='sim')sbRunSim();
}

function sbRunSim(){
  sbBuildCoef();
  var runs=parseInt((document.getElementById('sbRuns')||{}).value||'100',10);
  var tgt=Math.max(0.5,parseFloat((document.getElementById('sbTgt')||{}).value)||10);
  var aim=sbDrift(sbSegCross(sbNodes));
  var rng=sbRng(0x51D53);
  var xs=[],hits=0,half=tgt/2;
  for(var r=0;r<runs;r++){
    var pert=sbNodes.map(function(n){return {on:n.on,down:n.down,
      speed:Math.max(0,sbGauss(n.speed,n.spdSd,rng)),rel:sbGauss(n.rel,n.dirSd,rng)};});
    var resid=sbDrift(sbSegCross(pert))-aim;
    xs.push(resid); if(Math.abs(resid)<=half)hits++;
  }
  xs.sort(function(a,b){return a-b;});
  function q(p){return xs[Math.min(xs.length-1,Math.floor(p*xs.length))];}
  sbLastSim={xs:xs,hits:hits,runs:runs,pct:100*hits/runs,tgt:tgt,p05:q(0.05),p95:q(0.95)};
  var el;
  if(el=document.getElementById('sbHitPct'))el.textContent=sbLastSim.pct.toFixed(0)+'%';
  if(el=document.getElementById('sbCone'))el.textContent=(sbLastSim.p95-sbLastSim.p05).toFixed(1)+' in';
  sbDrawTarget();sbDrawHist();
}

// --- charts (raw canvas, no libs) ---
function sbDrawSens(){
  var c=document.getElementById('sbSensCv');if(!c)return;var g=c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);
  var mx=Math.max.apply(null,sbCoef.concat([1e-6])),bw=c.width/sbNSeg;
  for(var i=0;i<sbNSeg;i++){
    var h=(sbCoef[i]/mx)*(c.height-24);
    g.fillStyle='#ffb000';g.fillRect(i*bw+1,c.height-h-4,bw-2,h);
  }
  // mark active node positions
  g.fillStyle='#4cc2ff';
  sbNodes.filter(function(n){return n.on&&n.down>=0;}).forEach(function(n){
    var x=(n.down/sbRangeYd)*c.width;g.fillRect(x-1,0,2,c.height);
  });
  g.fillStyle='#8a94a6';g.font='10px sans-serif';
  g.fillText('muzzle',2,c.height-2);g.fillText('target',c.width-34,c.height-2);
}
function sbDrawTarget(){
  var c=document.getElementById('sbTargetCv');if(!c||!sbLastSim)return;var g=c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);var cx=c.width/2,cy=c.height/2;
  var scale=(c.width*0.44)/Math.max(sbLastSim.tgt*0.75,Math.abs(sbLastSim.p95),Math.abs(sbLastSim.p05),1);
  // target rings
  g.strokeStyle='#2a3340';g.lineWidth=1;
  [1,0.66,0.33].forEach(function(f){g.beginPath();g.arc(cx,cy,sbLastSim.tgt/2*scale*f,0,7);g.stroke();});
  g.strokeStyle='#39d98a';g.beginPath();g.arc(cx,cy,sbLastSim.tgt/2*scale,0,7);g.stroke();
  // impacts (vertical jitter is cosmetic; wind miss is horizontal)
  var jr=sbRng(7);
  sbLastSim.xs.forEach(function(x){
    var px=cx+x*scale, py=cy+(jr()-0.5)*sbLastSim.tgt*0.5*scale;
    var hit=Math.abs(x)<=sbLastSim.tgt/2;
    g.fillStyle=hit?'rgba(57,217,138,.7)':'rgba(255,107,107,.6)';
    g.fillRect(px-1.2,py-1.2,2.4,2.4);
  });
  g.fillStyle='#8a94a6';g.font='11px sans-serif';g.fillText('L',6,cy-4);g.fillText('R',c.width-14,cy-4);
}
function sbDrawHist(){
  var c=document.getElementById('sbHistCv');if(!c||!sbLastSim)return;var g=c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);
  var xs=sbLastSim.xs,lo=xs[0],hi=xs[xs.length-1];if(hi-lo<1e-6){hi=lo+1;}
  var B=31,bins=new Array(B).fill(0);
  xs.forEach(function(x){var b=Math.min(B-1,Math.floor((x-lo)/(hi-lo)*B));bins[b]++;});
  var mx=Math.max.apply(null,bins),bw=c.width/B;
  for(var i=0;i<B;i++){var h=(bins[i]/mx)*(c.height-18);
    var xc=lo+(i+0.5)/B*(hi-lo);var inTgt=Math.abs(xc)<=sbLastSim.tgt/2;
    g.fillStyle=inTgt?'#39d98a':'#ff6b6b';g.fillRect(i*bw+1,c.height-h-2,bw-1,h);}
  // center line (point of aim)
  var zx=(0-lo)/(hi-lo)*c.width;g.strokeStyle='#ffb000';g.beginPath();g.moveTo(zx,0);g.lineTo(zx,c.height);g.stroke();
}

// --- profiles (reuse solver's saved rifles) ---
// The Sandbox's profile pick is a SEPARATE choice from the Solver's activeProfile
// (see sbSensParams) and, unlike the Solver's pick, was never persisted anywhere --
// it lived only in the <select>'s live DOM value. sbRefreshProfiles() rebuilds
// that <select>'s innerHTML every time Sandbox is (re-)entered, which resets the
// browser's selection to blank. Net effect: pick a profile, leave the tab and
// come back (or the page reloads -- a WiFi hiccup, a backgrounded mobile browser
// tab getting reloaded) and the pick silently vanishes with no visible cause --
// Populate just looks permanently disabled. Persist it like the Solver does.
function sbActiveProfile(){return localStorage.getItem('sbActiveProfile')||'';}
function sbSetActiveProfile(n){localStorage.setItem('sbActiveProfile',n||'');}
function sbRefreshProfiles(){
  var sel=document.getElementById('sbProfileSel');if(!sel)return;
  var loads=window.Gear?Gear.loads():[];
  var h='<option value="">-- Solver profile (optional) --</option>';
  loads.forEach(function(l){h+='<option value="'+escHtml(l.id)+'">'+escHtml(Gear.label(l))+'</option>';});
  sel.innerHTML=h;
  // Picks saved before gear storage were profile NAMES; map one to its load id.
  var old=sbActiveProfile();
  if(old&&window.Gear&&!Gear.load(old)){var m=loadProfiles()[old];sbSetActiveProfile(m?m.loadId:'');}
  var all={};loads.forEach(function(l){all[l.id]=1;});
  // Restore the previously chosen profile, if it still exists. Setting .value
  // alone does not fire onchange, so drive the normal load path explicitly --
  // that's what re-populates the "Loaded: ..." summary and re-syncs Populate.
  var want=sbActiveProfile();
  if(want && all[want]!==undefined){ sel.value=want; sbLoadProfile(want); }
}
function sbLoadProfile(name){
  var box=document.getElementById('sbLoadInputs');if(!box)return;
  sbSetActiveProfile(name||'');
  if(!name){box.innerHTML='';sbInvalidateCurve();return;}
  var p=sbProfileFlat(name);
  if(p.range){document.getElementById('sbRange').value=p.range;}
  var show=['mv','bc','wgt','cal','range'];
  box.innerHTML='<div style="font-size:.76rem;color:var(--dim);margin-top:8px">Loaded: '+
    show.filter(function(k){return p[k]!==undefined;}).map(function(k){return k+' '+p[k];}).join(' &middot; ')+'</div>';
  sbInvalidateCurve();   // show approx now; user presses Populate for the true curve
}

// --- session save/load/export ---
function sbSnapshot(){return {v:1,range:document.getElementById('sbRange').value,
  segYd:document.getElementById('sbSegYd').value,tgt:document.getElementById('sbTgt').value,
  runs:document.getElementById('sbRuns').value,nodes:sbNodes};}
function sbApply(s){
  if(!s)return;
  if(s.range)document.getElementById('sbRange').value=s.range;
  if(s.segYd)document.getElementById('sbSegYd').value=s.segYd;
  if(s.tgt)document.getElementById('sbTgt').value=s.tgt;
  if(s.runs)document.getElementById('sbRuns').value=s.runs;
  if(s.nodes&&s.nodes.length)sbNodes=s.nodes.map(function(n,i){return {id:n.id||i+1,on:!!n.on,
    down:+n.down,speed:+n.speed,rel:+n.rel,spdSd:+n.spdSd,dirSd:+n.dirSd};});
  sbRenderNodes();sbRecalc();
}
function sbSessions(){try{return JSON.parse(localStorage.getItem('sbSessions')||'{}');}catch(e){return {};}}
function sbStoreSessions(o){localStorage.setItem('sbSessions',JSON.stringify(o));}
function sbRefreshSessions(){
  var sel=document.getElementById('sbSessionSel');if(!sel)return;
  var all=sbSessions();var h='<option value="">-- Saved sessions --</option>';
  Object.keys(all).forEach(function(k){h+='<option>'+k+'</option>';});sel.innerHTML=h;
}
function sbSaveSession(){var n=prompt('Session name:','sandbox 1');if(!n)return;
  var all=sbSessions();all[n.trim()]=sbSnapshot();sbStoreSessions(all);sbRefreshSessions();flash('Saved "'+n.trim()+'"');}
function sbLoadSession(name){if(!name)return;var all=sbSessions();if(all[name])sbApply(all[name]);}
function sbLoadSessionPrompt(){sbRefreshSessions();flash('Pick a session from the dropdown');}
function sbExportSession(){
  var blob=new Blob([JSON.stringify(sbSnapshot(),null,2)],{type:'application/json'});
  var a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='sandbox_session.json';a.click();
}
function sbImportSession(inp){
  var f=inp.files[0];if(!f)return;var rd=new FileReader();
  rd.onload=function(){try{sbApply(JSON.parse(rd.result));flash('Session imported');}catch(e){flash('Bad file');}};
  rd.readAsText(f);inp.value='';
}

function sbSetSub(s){
  sbSub=s;
  document.getElementById('sbTabSetup').className='tab'+(s==='setup'?' on':'');
  document.getElementById('sbTabSim').className='tab'+(s==='sim'?' on':'');
  document.getElementById('sbSetup').className=(s==='setup')?'':'hide';
  document.getElementById('sbSim').className=(s==='sim')?'':'hide';
  if(s==='sim'){sbRunSim();sbSyncPopulateBtn(false);}
}
function sbShow(){
  // Suspend the live-data poll while in the Sandbox. The Sandbox is an offline
  // virtual environment and needs nothing from /data.json. More importantly,
  // windSensitivity() on the ESP32 takes 2-4 s (software double FP) and blocks
  // the HTTP server -- simultaneous poll() requests pile up behind it and slow
  // the response. Giving the server sole attention cuts the round-trip noticeably.
  // setSection() restarts the poll the moment the user leaves this tab.
  if(pollTimer){ clearInterval(pollTimer); pollTimer=null; }
  sbInit();sbRefreshProfiles();sbRefreshSessions();sbRenderNodes();sbRecalc();
  sbInvalidateCurve();   // start on the approximate curve; Populate pulls the true one
}

// ---- press feedback: a brief scale pulse on any tap so a press is felt ----
// Delegated + pointerdown-driven so it fires reliably on touch (iOS :active is
// unreliable). Disabled controls don't emit pointer events, so they're skipped.
function wirePressFx(){
  document.addEventListener('pointerdown',function(e){
    var el=e.target.closest('button,.tab,.sec,.step,.solveBtn');
    if(!el||el.disabled)return;
    el.classList.remove('pressed');void el.offsetWidth;   // restart the animation
    el.classList.add('pressed');
  },true);
  document.addEventListener('animationend',function(e){
    if(e.animationName==='btnPress')e.target.classList.remove('pressed');
  },true);
}
wirePressFx();
// Gear (rifles/loads/bullets) lives on the base station; this phone keeps a copy.
// Pull/push on load, when the page comes back to the front, and every 30 s.
function startGear(){
  if(!window.Gear)return;
  followActiveLoad();
  Gear.onChange(function(){
    if(followActiveLoad()&&view==='solver'){renderSolver();return;}
    if(view==='solver')renderGearLists();
  });
  Gear.sync();
  setInterval(function(){if(!document.hidden)Gear.sync();},30000);
  document.addEventListener('visibilitychange',function(){if(!document.hidden)Gear.sync();});
  if(Gear.migrated)flash(Gear.migrated+' rifle profile(s) moved to the new gear store');
}
startGear();
setSection(section);saveAz();drawWheel();poll();pollTimer=setInterval(poll,350);
</script></body></html>
)HTMLPAGE";

class WebUI {
private:
    WebServer server{80};
    String (*jsonProvider)() = nullptr;
    String (*solveProvider)(WebServer&) = nullptr;
    String (*sensProvider)(WebServer&) = nullptr;
    String (*cmdProvider)(WebServer&) = nullptr;
    String (*aimProvider)(WebServer&) = nullptr;
    void   (*tickFn)() = nullptr;          // keeps the radio serviced during long transfers

    // ---- gear store (LittleFS) ----
    bool     fsOk = false;
    uint32_t gearVer = 0;                  // bumps on every accepted save
    bool     gearGz = false;               // stored bytes are gzip
    // upload in progress
    File     upFile;
    size_t   upBytes = 0;
    int      upStatus = 0;                 // 0 ok so far, else HTTP status to answer
    bool     upSeen = false;               // a file part actually arrived
    bool     upGz = false;


    void tick() { if (tickFn) tickFn(); }

    // File present? Uses stat() because on core 2.0.x LittleFS.open()/exists()
    // print an [E] log line for every missing file.
    static bool fsHas(const char* path) {
        struct stat st;
        return stat((String("/littlefs") + path).c_str(), &st) == 0;
    }
    void loadMeta() {
        gearVer = 0; gearGz = false;
        if (!fsHas(GEAR_META)) return;
        File m = LittleFS.open(GEAR_META, "r");
        if (!m) return;
        String line = m.readStringUntil('\n');
        m.close();
        int sp = line.indexOf(' ');
        gearVer = (uint32_t)line.substring(0, sp < 0 ? line.length() : sp).toInt();
        gearGz  = (sp >= 0 && line.substring(sp + 1).toInt() == 1);
    }
    // At boot: a power cut between saving the file and saving gear.meta would leave
    // the old version number and gzip flag beside the new file. Take the gzip flag
    // from the file itself, and move to a fresh version number so no phone takes
    // the new file for one it already has.
    void checkGearAtBoot() {
        loadMeta();
        if (!fsHas(GEAR_PATH)) return;
        File f = LittleFS.open(GEAR_PATH, "r");
        if (!f) return;
        uint8_t m[2] = {0, 0};
        size_t n = f.read(m, 2);
        f.close();
        gearGz = (n == 2 && m[0] == 0x1f && m[1] == 0x8b);
        gearVer++;
        saveMeta();
    }
    bool saveMeta() {
        File m = LittleFS.open(GEAR_META, "w");
        if (!m) return false;
        m.printf("%lu %d\n", (unsigned long)gearVer, gearGz ? 1 : 0);
        m.close();
        return true;
    }
    size_t gearSize() {
        if (!fsOk || !fsHas(GEAR_PATH)) return 0;
        File f = LittleFS.open(GEAR_PATH, "r");
        size_t n = f ? f.size() : 0;
        if (f) f.close();
        return n;
    }
    String gearInfoJson() {
        String j = "{\"ok\":" + String(fsOk ? "true" : "false");
        j += ",\"ver\":" + String(gearVer) + ",\"bytes\":" + String(gearSize());
        j += ",\"gz\":" + String(gearGz ? "true" : "false") + ",\"max\":" + String((unsigned long)GEAR_MAX_BYTES);
        if (fsOk) j += ",\"fsTotal\":" + String((unsigned long)LittleFS.totalBytes()) +
                       ",\"fsUsed\":" + String((unsigned long)LittleFS.usedBytes());
        return j + "}";
    }

    void sendGear() {
        server.sendHeader("Cache-Control", "no-store");
        if (!fsOk) { server.send(503, "application/json", "{\"ok\":false,\"err\":\"fs\"}"); return; }
        File f = fsHas(GEAR_PATH) ? LittleFS.open(GEAR_PATH, "r") : File();
        if (!f || f.size() == 0) { if (f) f.close(); server.send(204); return; }
        server.sendHeader("X-Gear-Ver", String(gearVer));
        server.sendHeader("Access-Control-Expose-Headers", "X-Gear-Ver");
        if (gearGz) server.sendHeader("Content-Encoding", "gzip");
        server.setContentLength(f.size());
        server.send(200, "application/json", "");
        static uint8_t buf[2048];
        while (f.available()) {
            size_t n = f.read(buf, sizeof(buf));
            if (n == 0) break;
            server.sendContent((const char*)buf, n);
            tick();
        }
        f.close();
    }

    // multipart chunks for POST /gear
    void onGearUpload() {
        HTTPUpload& up = server.upload();
        if (up.status == UPLOAD_FILE_START) {
            upBytes = 0; upStatus = 0; upSeen = true;
            upGz = (server.arg("gz") == "1");
            if (!fsOk) { upStatus = 503; return; }
            if (server.hasArg("base") && (uint32_t)server.arg("base").toInt() != gearVer) { upStatus = 409; return; }
            if (fsHas(GEAR_TMP)) LittleFS.remove(GEAR_TMP);
            upFile = LittleFS.open(GEAR_TMP, "w");
            if (!upFile) upStatus = 500;
        } else if (up.status == UPLOAD_FILE_WRITE) {
            if (upStatus) return;
            if (upBytes + up.currentSize > GEAR_MAX_BYTES) {
                upStatus = 413; upFile.close(); LittleFS.remove(GEAR_TMP); return;
            }
            if (upFile.write(up.buf, up.currentSize) != up.currentSize) {
                upStatus = 507; upFile.close(); LittleFS.remove(GEAR_TMP); return;   // flash full
            }
            upBytes += up.currentSize;
            tick();
        } else if (up.status == UPLOAD_FILE_END) {
            if (upStatus) return;
            upFile.close();
            if (upBytes == 0) { upStatus = 400; LittleFS.remove(GEAR_TMP); return; }
            // LittleFS renames over an existing file atomically; the remove is
            // only a fallback for a VFS layer that refuses to replace.
            if (!LittleFS.rename(GEAR_TMP, GEAR_PATH)) {
                if (fsHas(GEAR_PATH)) LittleFS.remove(GEAR_PATH);
                if (!LittleFS.rename(GEAR_TMP, GEAR_PATH)) { upStatus = 500; return; }
            }
            gearVer++; gearGz = upGz;
            if (!saveMeta()) upStatus = 500;
            Serial.printf("[gear] saved v%lu, %u bytes%s\n", (unsigned long)gearVer, (unsigned)upBytes, upGz ? " (gzip)" : "");
        } else if (up.status == UPLOAD_FILE_ABORTED) {
            if (upFile) upFile.close();
            LittleFS.remove(GEAR_TMP);
            upStatus = 400;
        }
    }
    void finishGearUpload() {
        server.sendHeader("Cache-Control", "no-store");
        if (!upSeen) upStatus = 400;           // POST with no file part
        int code = upStatus ? upStatus : 200;
        String err = code == 409 ? "conflict" : code == 413 ? "too_big" : code == 507 ? "flash_full" :
                     code == 503 ? "fs" : code == 400 ? "empty" : "write";
        String j = (code == 200) ? "{\"ok\":true,\"ver\":" + String(gearVer) + ",\"bytes\":" + String((unsigned)upBytes) + "}"
                                 : "{\"ok\":false,\"err\":\"" + err + "\",\"ver\":" + String(gearVer) + "}";
        server.send(code, "application/json", j);
        upStatus = 0; upSeen = false;
    }

public:
    // tick: optional, called every ~2 KB during gear transfers so the receiver
    // keeps draining the radio and sending beacons while a big file moves.
    void begin(const char* ssid, const char* pass,
               String (*jprov)(), String (*sprov)(WebServer&),
               String (*senprov)(WebServer&) = nullptr,
               String (*cprov)(WebServer&) = nullptr,
               String (*aprov)(WebServer&) = nullptr,
               void (*tickCb)() = nullptr) {
        jsonProvider = jprov; solveProvider = sprov; sensProvider = senprov;
        cmdProvider = cprov; aimProvider = aprov; tickFn = tickCb;

        // Data partition "spiffs" in huge_app.csv, mounted as LittleFS. First
        // boot (blank or never-formatted partition) formats it once.
        fsOk = LittleFS.begin(true);
        if (fsOk) {
            checkGearAtBoot();
            if (fsHas(GEAR_TMP)) LittleFS.remove(GEAR_TMP);   // half-finished upload from before a reset
            Serial.printf("[gear] flash store ready: v%lu, %u bytes (%u of %u KB used)\n",
                          (unsigned long)gearVer, (unsigned)gearSize(),
                          (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024));
        } else {
            Serial.println(F("[gear] LittleFS mount FAILED -- gear stays on each phone only"));
        }

        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid, pass);
        IPAddress ip = WiFi.softAPIP();
        server.on("/", [this]() { server.send_P(200, "text/html", WEBUI_PAGE); });
        server.on("/engine.js", [this]() {
            server.sendHeader("Cache-Control", "no-cache");
            server.send_P(200, "application/javascript", ENGINE_JS);
        });
        server.on("/gear.js", [this]() {
            server.sendHeader("Cache-Control", "no-cache");
            server.send_P(200, "application/javascript", GEAR_JS);
        });
        server.on("/sky.js", [this]() {
            server.sendHeader("Cache-Control", "no-cache");
            server.send_P(200, "application/javascript", SKY_JS);
        });
        server.on("/gear", HTTP_GET, [this]() { sendGear(); });
        server.on("/gear", HTTP_POST, [this]() { finishGearUpload(); }, [this]() { onGearUpload(); });
        server.on("/gear/info", HTTP_GET, [this]() {
            server.sendHeader("Cache-Control", "no-store");
            server.send(200, "application/json", gearInfoJson());
        });
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
        server.on("/cmd", [this]() {
            server.sendHeader("Cache-Control","no-store");
            server.send(200, "application/json", cmdProvider ? cmdProvider(server) : "{\"ok\":false}");
        });
        server.on("/aim", [this]() {
            server.sendHeader("Cache-Control","no-store");
            server.send(200, "application/json", aimProvider ? aimProvider(server) : "{\"ok\":false}");
        });
        server.onNotFound([this]() { server.send_P(200, "text/html", WEBUI_PAGE); });
        server.begin();
        Serial.printf("[WebUI] AP '%s'  ->  http://%s\n", ssid, ip.toString().c_str());
    }
    void handle() { server.handleClient(); }
};

#endif // WEBUI_H
