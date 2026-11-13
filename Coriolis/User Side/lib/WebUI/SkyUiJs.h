// SkyUiJs.h -- /sky.js: the Skyward Ballistics look and feel for the Solver tab.
//
// A full-screen app inside Wind Lab: Calculator, My gear, Analysis and More
// tabs, the Solution / Table / Target Card / Calibrate screens, Skyward's
// colours, wind dial and spinning logo. It solves with SkyEngine (/engine.js)
// and keeps gear in Gear (/gear.js), so it reads and writes the same profile,
// live-wind switches, firing azimuth and group offsets as the rest of Wind Lab:
// WIND HOLD, the aimbot, the log, auto weights and the sandbox all agree with it.
// Opened by setView('solver'); More -> Classic Coriolis solver shows the old form.
#pragma once
#include <Arduino.h>

static const char SKY_JS[] PROGMEM = R"SKYJS(
(function(root){
'use strict';
var doc=root.document;
if(!doc)return;

// ============================================================================
// Look: Skyward's Material 3 theme (res/values/calc_colors.xml, calc_theme.xml)
// ============================================================================
var CSS=[
'#sky{--navy:#01274A;--cyan:#01D1FF;--cream:#FCF9F5;',
' --primary:#01274A;--on-primary:#FFFFFF;--pc:#BDEFFF;--on-pc:#01274A;--secondary:#005F80;--sc:#CAF5FE;--on-sc:#01274A;',
' --bg:#FCF9F5;--on:#01274A;--on-var:#3A4D66;--s-lowest:#FFFFFF;--s-low:#F8F4EE;--s-cont:#F3EFE8;--s-high:#FFFFFF;--s-highest:#E6ECF2;',
' --outline:#5A6B80;--outline-var:#CCD3DB;--inv:#01274A;--on-inv:#FCF9F5;--error:#B00020;',
' --accent:#01D1FF;--on-accent:#01274A;--accent-stroke:#01274A;--toolbar:#01274A;--on-toolbar:#FCF9F5;',
' --elev:#0A4A7A;--wind:#8A3B00;--lead:#1B5E20;--wind2:#6D3A1F;--warn:#B00020;--live:#1B7F4B}',
'#sky.dark{--primary:#01D1FF;--on-primary:#01274A;--pc:#0B4A6B;--on-pc:#D6F6FF;--secondary:#7FE3FF;--sc:#124A6E;--on-sc:#D6F6FF;',
' --bg:#04182E;--on:#FCF9F5;--on-var:#C5D0DC;--s-lowest:#03111F;--s-low:#071F3A;--s-cont:#0A2644;--s-high:#0E2F52;--s-highest:#163B62;',
' --outline:#8C9BAD;--outline-var:#2B4666;--inv:#FCF9F5;--on-inv:#01274A;--error:#FFB4AB;--accent-stroke:#01D1FF;',
' --elev:#5CDDFF;--wind:#FFB870;--lead:#8EE59B;--wind2:#F0B79A;--warn:#FFB4AB;--live:#7BE0A6}',
'#sky{position:fixed;inset:0;z-index:1000;display:flex;flex-direction:column;background:var(--bg);color:var(--on);',
' font-family:Roboto,"Google Sans",system-ui,-apple-system,"Segoe UI",sans-serif;font-size:17px;line-height:1.3;',
' -webkit-tap-highlight-color:transparent;-webkit-text-size-adjust:100%;overscroll-behavior:contain}',
'#sky *{box-sizing:border-box}',
'#sky.hide{display:none}',
// Wind Lab's own page styles use short class names (.v .k .big .warn) and a bare svg rule; keep them out of here.
'#sky *{font-family:inherit;text-transform:inherit;letter-spacing:inherit}',
'#sky svg{max-width:none;margin:0}',
'#sky button{font:inherit;color:inherit;transform:none;animation:none}',
'#sky svg.sk-ic{display:inline-block;width:24px;height:24px;max-width:none;margin:0;fill:currentColor;flex:0 0 auto}',
// toolbar
'#sky .sk-bar{flex:0 0 auto;display:flex;align-items:center;min-height:64px;background:var(--toolbar);color:var(--on-toolbar);padding:0 4px 0 4px;gap:4px}',
'#sky .sk-bar .sk-title{flex:1;font-size:22px;font-weight:700;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;padding-left:12px}',
'#sky .sk-bar .sk-title.word{font-size:20px;letter-spacing:.14em;padding-left:12px}',
'#sky .sk-ibtn{width:48px;height:48px;border:0;background:transparent;border-radius:24px;display:flex;align-items:center;justify-content:center;cursor:pointer;color:inherit;padding:0}',
'#sky .sk-ibtn:active{background:rgba(127,127,127,.18)}',
'#sky .sk-logo{position:relative;width:34px;height:34px;margin-left:12px;flex:0 0 auto}',
'#sky .sk-logo img{position:absolute;inset:0;width:100%;height:100%}',
'#sky .sk-logo .rif{animation:skSpin 24s linear infinite}',
'#sky.still .sk-logo .rif,#sky.still .sk-busy .rif{animation:none}',
'@keyframes skSpin{to{transform:rotate(360deg)}}',
// body
'#sky .sk-main{flex:1;position:relative;overflow:hidden}',
'#sky .sk-scroll{position:absolute;inset:0;overflow-y:auto;-webkit-overflow-scrolling:touch}',
'#sky .sk-col{max-width:640px;margin:0 auto;padding-bottom:24px}',
'#sky .sk-in{animation:skIn .18s ease-out}',
'#sky .sk-in.back{animation:skInB .18s ease-out}',
'#sky.still .sk-in{animation:none}',
'@keyframes skIn{from{opacity:.35;transform:translateX(18px)}to{opacity:1;transform:none}}',
'@keyframes skInB{from{opacity:.35;transform:translateX(-18px)}to{opacity:1;transform:none}}',
// form: header / note / group
'#sky .sk-h{display:flex;align-items:center;padding:20px 12px 8px 16px;min-height:52px}',
'#sky .sk-h .bar{width:5px;height:20px;background:var(--cyan);margin-right:8px;flex:0 0 auto}',
'#sky .sk-h .t{flex:1;text-transform:uppercase;letter-spacing:.06em;font-size:15px;font-weight:700;color:var(--on)}',
'#sky .sk-note{font-size:16px;line-height:1.4;color:var(--on-var);padding:8px 20px 4px;white-space:pre-line}',
'#sky .sk-group{margin:0 12px;border:1px solid var(--outline-var);border-radius:16px;background:var(--s-high);overflow:hidden}',
'#sky .sk-gap{height:10px}',
'#sky .sk-div{height:1px;background:var(--outline-var);margin:0 16px}',
'#sky .sk-row{display:flex;align-items:center;min-height:60px;padding:8px 16px;gap:0;position:relative}',
'#sky .sk-row.v{flex-direction:column;align-items:stretch}',
'#sky .sk-row.tap{cursor:pointer}',
'#sky .sk-row.tap:active,#sky .sk-item:active,#sky .sk-act:active{background:rgba(127,127,127,.12)}',
'#sky .sk-line{display:flex;align-items:center;width:100%}',
'#sky .sk-lab{flex:1;min-width:0;display:flex;flex-direction:column}',
'#sky .sk-lab .l{font-size:18px;color:var(--on)}',
'#sky .sk-lab .l.b{font-weight:700}',
'#sky .sk-lab .s{font-size:15px;color:var(--on-var);margin-top:1px}',
'#sky .sk-val{font-size:18px;font-weight:700;text-align:right;margin-left:12px;max-width:56%;color:var(--on);overflow-wrap:anywhere}',
'#sky .sk-val.n{font-weight:400}',
'#sky .sk-val.warn{color:var(--warn)}',
'#sky .sk-chev{width:28px;height:28px;margin-left:4px;color:var(--on-var)}',
'#sky .sk-chev.down{transform:rotate(90deg)}',
'#sky .sk-num{background:var(--s-lowest);border:1.5px solid var(--outline);border-radius:10px;color:var(--on);',
' font:700 20px/1.2 inherit;font-family:inherit;text-align:center;min-width:84px;width:96px;min-height:48px;padding:6px 10px;margin-left:12px;outline:none;-moz-appearance:textfield}',
'#sky .sk-num::-webkit-outer-spin-button,#sky .sk-num::-webkit-inner-spin-button{-webkit-appearance:none;margin:0}',
'#sky .sk-num:focus,#sky .sk-txt:focus{border:2.5px solid var(--cyan);padding:5px 9px}',
'#sky .sk-num.big{font-size:30px;min-width:120px;width:132px;min-height:60px;margin:0}',
'#sky .sk-num:disabled{opacity:1;color:var(--live);border-style:dashed}',
'#sky .sk-txt{background:var(--s-lowest);border:1.5px solid var(--outline);border-radius:10px;color:var(--on);font-size:20px;font-family:inherit;',
' width:100%;min-height:52px;padding:8px 12px;outline:none;margin-top:6px}',
'#sky textarea.sk-txt{min-height:96px;font-size:17px;resize:vertical}',
'#sky .sk-unit{font-size:16px;color:var(--on-var);padding-left:6px;min-width:30px}',
'#sky .sk-unit.big{font-size:18px}',
'#sky .sk-steprow{display:flex;align-items:center;margin-top:8px}',
'#sky .sk-steprow .mid{flex:1;display:flex;align-items:center;justify-content:center}',
'#sky .sk-step{width:60px;height:60px;border-radius:30px;border:2px solid var(--on);background:transparent;color:var(--on);',
' font-size:30px;font-weight:700;display:flex;align-items:center;justify-content:center;cursor:pointer;flex:0 0 auto;padding:0;line-height:1}',
'#sky .sk-step:active,#sky .sk-btn:active,#sky .sk-obtn:active,#sky .sk-pill:active{transform:scale(.96)}',
'#sky.still .sk-step:active,#sky.still .sk-btn:active,#sky.still .sk-obtn:active,#sky.still .sk-pill:active{transform:none}',
'#sky .sk-step:disabled{opacity:.3}',
'#sky .sk-pm{min-width:60px;height:48px;margin:0 8px;border:1.5px solid var(--outline);border-radius:24px;background:transparent;color:var(--primary);font-size:18px;cursor:pointer}',
'#sky .sk-pill{min-height:48px;border:0;border-radius:24px;background:var(--sc);color:var(--on-sc);font-size:16px;padding:4px 16px;cursor:pointer;font-weight:500}',
'#sky .sk-under{display:flex;justify-content:flex-end;margin-top:8px;gap:8px;flex-wrap:wrap}',
// switch (Material 3)
'#sky .sk-sw{position:relative;width:52px;height:32px;flex:0 0 auto;margin-left:12px;border-radius:16px;border:2px solid var(--outline);background:var(--s-highest);transition:background .15s,border-color .15s;cursor:pointer}',
'#sky .sk-sw::after{content:"";position:absolute;top:6px;left:6px;width:16px;height:16px;border-radius:50%;background:var(--outline);transition:all .15s}',
'#sky .sk-sw.on{background:var(--primary);border-color:var(--primary)}',
'#sky .sk-sw.on::after{top:2px;left:22px;width:24px;height:24px;background:var(--on-primary)}',
// segmented
'#sky .sk-seg{min-width:64px;min-height:48px;margin-left:6px;padding:0 14px;border:1.5px solid var(--outline);border-radius:24px;background:transparent;color:var(--on);font-size:17px;cursor:pointer}',
'#sky .sk-seg.on{background:var(--sc);color:var(--on-sc);font-weight:700}',
// fold
'#sky .sk-fold{min-height:68px}',
'#sky .sk-fold .hint{font-size:16px;font-weight:700;color:var(--secondary);padding-left:8px;min-width:52px;text-align:right}',
'#sky .sk-fold .sk-chev{color:var(--secondary);width:30px;height:30px;transition:transform .22s}',
'#sky .sk-fold .sum{overflow:hidden;transition:opacity .22s}',
'#sky .sk-body{overflow:hidden}',
// buttons
'#sky .sk-btn{display:flex;align-items:center;justify-content:center;gap:8px;width:100%;min-height:56px;border-radius:28px;border:1.5px solid var(--accent-stroke);',
' background:var(--accent);color:var(--on-accent);font-size:18px;font-weight:700;letter-spacing:.01em;cursor:pointer;padding:0 20px}',
'#sky .sk-btn:disabled{background:rgba(127,127,127,.14);border-color:rgba(127,127,127,.2);color:rgba(127,127,127,.6)}',
'#sky .sk-obtn{display:flex;align-items:center;justify-content:center;gap:8px;min-height:52px;border-radius:26px;border:1.5px solid var(--outline);background:transparent;color:var(--primary);font-size:18px;cursor:pointer;padding:0 18px}',
'#sky .sk-tbtn{min-height:48px;border:0;background:transparent;color:var(--primary);font-size:17px;font-weight:700;cursor:pointer;padding:0 12px;border-radius:24px}',
'#sky .sk-bwrap{padding:12px 12px 4px}',
'#sky .sk-brow{display:flex;gap:8px;padding:12px 12px 4px}',
'#sky .sk-brow>*{flex:1}',
// result bar (calculator)
'#sky .sk-rule{flex:0 0 auto;height:3px;background:var(--cyan)}',
'#sky .sk-res{flex:0 0 auto;background:var(--s-high);padding:8px 12px 10px}',
'#sky .sk-res .inner{max-width:640px;margin:0 auto}',
'#sky .sk-cells{display:flex}',
'#sky .sk-cell{flex:1;display:flex;flex-direction:column;align-items:center;padding:0 4px;min-width:0}',
'#sky .sk-cell .c{font-size:14px;letter-spacing:.08em;font-weight:700;color:var(--on-var)}',
'#sky .sk-cell .vl{display:flex;align-items:baseline;justify-content:center}',
'#sky .sk-cell .v{font-size:36px;font-weight:700;line-height:1.05;white-space:nowrap}',
'#sky .sk-cell .u{font-size:16px;font-weight:700;padding-left:6px;color:var(--on)}',
'#sky .sk-cell .s{font-size:16px;color:var(--on);text-align:center}',
'#sky .sk-hint{text-align:center;font-size:18px;color:var(--on);padding:6px 0}',
'#sky .sk-hint.small{font-size:15px}',
'#sky .sk-res .sk-btn{min-height:60px;font-size:20px;margin-top:6px}',
// bottom navigation
'#sky .sk-nav{flex:0 0 auto;display:flex;background:var(--s-high);border-top:1px solid var(--outline-var);min-height:80px;padding:12px 0 14px}',
'#sky .sk-nav .it{flex:1;display:flex;flex-direction:column;align-items:center;gap:4px;border:0;background:transparent;cursor:pointer;color:var(--on-var);padding:0;font-size:14px}',
'#sky .sk-nav .ind{width:64px;height:32px;border-radius:16px;display:flex;align-items:center;justify-content:center;transition:background .18s}',
'#sky .sk-nav .ind svg.sk-ic{width:28px;height:28px}',
'#sky .sk-nav .it.on{color:var(--on);font-weight:700}',
'#sky .sk-nav .it.on .ind{background:var(--cyan);color:var(--on-accent)}',
// gear list items
'#sky .sk-item{display:flex;align-items:center;min-height:76px;padding:10px 0 10px 16px;cursor:pointer}',
'#sky .sk-item .ck{width:30px;height:30px;margin-right:12px;color:var(--secondary);visibility:hidden;flex:0 0 auto}',
'#sky .sk-item .ck.on{visibility:visible}',
'#sky .sk-item .tx{flex:1;min-width:0}',
'#sky .sk-item .t{font-size:20px;font-weight:700;color:var(--on);overflow-wrap:anywhere}',
'#sky .sk-item .t.sm{font-size:18px}',
'#sky .sk-item .s{font-size:16px;color:var(--on-var);margin-top:2px}',
'#sky .sk-item .mo{width:56px;height:56px;border:0;background:transparent;color:var(--on-var);display:flex;align-items:center;justify-content:center;cursor:pointer;border-radius:28px}',
'#sky .sk-card{margin:4px 12px 8px;border:1px solid var(--outline-var);border-radius:16px;background:var(--s-high);overflow:hidden}',
'#sky .sk-act{display:flex;align-items:center;min-height:56px;padding:0 16px 0 40px;font-size:18px;font-weight:700;color:var(--secondary);cursor:pointer}',
'#sky .sk-tag{display:inline-block;font-size:13px;font-weight:700;border-radius:10px;padding:2px 8px;margin-left:8px;background:var(--sc);color:var(--on-sc);vertical-align:middle}',
// solution screen
'#sky .sk-chips{display:flex;padding:12px 8px 6px}',
'#sky .sk-chip{flex:1;margin:0 4px;border-radius:12px;background:var(--sc);color:var(--on-sc);padding:6px;text-align:center}',
'#sky .sk-chip .k{font-size:15px}',
'#sky .sk-chip .v{font-size:18px;font-weight:700}',
'#sky .sk-sub{text-align:center;font-size:17px;color:var(--on-var);padding:0 16px 6px}',
'#sky .sk-grid{display:flex;flex-wrap:wrap;padding:0 8px}',
'#sky .sk-grid>div{flex:1 1 50%;padding:4px}',
'#sky .sk-big{border:1px solid var(--outline-var);border-radius:14px;background:var(--s-high);overflow:hidden;text-align:center}',
'#sky .sk-big .stripe{height:6px}',
'#sky .sk-big .c{font-size:15px;letter-spacing:.08em;font-weight:700;color:var(--on-var);padding:8px 4px 0}',
'#sky .sk-big .vl{display:flex;align-items:baseline;justify-content:center;padding:0 6px}',
'#sky .sk-big .v{font-size:40px;font-weight:700;line-height:1.1}',
'#sky .sk-big .u{font-size:16px;font-weight:700;padding-left:6px}',
'#sky .sk-big .s{font-size:18px;padding:2px 4px 10px}',
'#sky .sk-sstep .lab{font-size:16px;color:var(--on-var)}',
'#sky .sk-sstep .row{display:flex;align-items:center}',
'#sky .sk-sstep .val{flex:1;text-align:center;font-size:22px;font-weight:700}',
// table
'#sky .sk-tscroll{overflow-x:auto;-webkit-overflow-scrolling:touch}',
'#sky table.sk-tbl{border-collapse:collapse;font-size:15px;min-width:100%}',
'#sky table.sk-tbl td,#sky table.sk-tbl th{padding:5px 4px;text-align:center;white-space:nowrap}',
'#sky table.sk-tbl th{font-weight:700;background:var(--s-highest);white-space:pre-line;line-height:1.2}',
'#sky table.sk-tbl tr.st td{background:var(--s-cont)}',
'#sky table.sk-tbl tr.tg td{background:var(--pc);color:var(--on-pc);font-weight:700}',
'#sky table.sk-tbl td.l{text-align:left}',
'#sky .sk-blk{font-weight:700;text-align:center;background:var(--s-highest);padding:4px;font-size:15px;text-transform:uppercase}',
'#sky .sk-kv{display:grid;grid-template-columns:auto 1fr auto 1fr;gap:0 8px;padding:4px 8px;font-size:15px}',
'#sky .sk-kv b{font-weight:700}',
// target card rows
'#sky .sk-trow{display:flex;align-items:center;width:100%;padding:4px 0;cursor:pointer}',
'#sky .sk-trow .n{font-size:18px;font-weight:700}',
'#sky .sk-trow .d{font-size:15px;opacity:.9}',
'#sky .sk-trow .r{text-align:right;font-family:ui-monospace,"Roboto Mono",Menlo,monospace;font-weight:700;font-size:18px;white-space:pre-line;margin-left:8px}',
// wind dial
'#sky .sk-dialbox{display:flex;flex-direction:column;align-items:center;width:100%}',
'#sky .sk-dialbox .top{font-size:15px;color:var(--on-var)}',
'#sky .sk-dial{width:176px;height:176px;margin:6px 0 4px;touch-action:none;cursor:pointer}',
'#sky .sk-dial.off{cursor:default;opacity:.92}',
'#sky .sk-dialbox .hr{font-size:22px;font-weight:700}',
'#sky .sk-dialbox .sd{font-size:16px}',
'#sky .sk-dialbox .tip{font-size:14px;color:var(--on-var);padding-top:4px;text-align:center}',
'#sky .sk-live{display:inline-flex;align-items:center;gap:6px;font-size:13px;font-weight:700;letter-spacing:.06em;color:var(--live);border:1.5px solid var(--live);border-radius:12px;padding:1px 8px;margin-left:8px;vertical-align:middle}',
'#sky .sk-live i{width:7px;height:7px;border-radius:50%;background:var(--live);animation:skPulse 1.6s ease-in-out infinite}',
'@keyframes skPulse{50%{opacity:.25}}',
// sheets, dialogs, snackbar
'#sky .sk-scrim{position:absolute;inset:0;background:rgba(0,0,0,.42);z-index:20;display:flex;align-items:center;justify-content:center;padding:24px;animation:skFade .15s}',
'#sky .sk-scrim.sheet{align-items:flex-end;padding:0}',
'@keyframes skFade{from{opacity:0}}',
'#sky .sk-dlg{background:var(--s-high);color:var(--on);border-radius:28px;padding:24px 0 12px;width:100%;max-width:440px;max-height:86%;display:flex;flex-direction:column;box-shadow:0 8px 32px rgba(0,0,0,.3)}',
'#sky .sk-dlg .dt{font-size:24px;padding:0 24px 16px}',
'#sky .sk-dlg .db{font-size:18px;padding:0 24px 8px;overflow-y:auto;white-space:pre-line;line-height:1.4}',
'#sky .sk-dlg .dl{overflow-y:auto}',
'#sky .sk-dlg .ch{display:flex;align-items:center;min-height:56px;padding:0 24px;font-size:19px;cursor:pointer;gap:16px}',
'#sky .sk-dlg .ch:active{background:rgba(127,127,127,.12)}',
'#sky .sk-radio{width:20px;height:20px;border-radius:50%;border:2px solid var(--on-var);flex:0 0 auto;position:relative}',
'#sky .sk-radio.on{border-color:var(--primary)}',
'#sky .sk-radio.on::after{content:"";position:absolute;inset:3px;border-radius:50%;background:var(--primary)}',
'#sky .sk-dlg .da{display:flex;justify-content:flex-end;flex-wrap:wrap;gap:4px;padding:12px 16px 0}',
'#sky .sk-dlg .da button{min-height:52px;font-size:18px}',
'#sky .sk-dlg .sk-txt{margin:0}',
'#sky .sk-sheet{background:var(--s-high);border-radius:28px 28px 0 0;width:100%;max-width:640px;max-height:86%;overflow-y:auto;padding:4px 0 16px;animation:skUp .2s ease-out}',
'@keyframes skUp{from{transform:translateY(40px);opacity:.4}}',
'#sky .sk-sheet .grab{width:32px;height:4px;border-radius:2px;background:var(--outline);opacity:.5;margin:12px auto 4px}',
'#sky .sk-sheet .st{font-size:20px;font-weight:700;padding:8px 20px}',
'#sky .sk-menu{position:absolute;z-index:30;background:var(--s-high);border-radius:8px;box-shadow:0 6px 24px rgba(0,0,0,.28);min-width:200px;padding:8px 0}',
'#sky .sk-menu div{padding:0 16px;min-height:52px;display:flex;align-items:center;font-size:18px;cursor:pointer}',
'#sky .sk-menu div:active{background:rgba(127,127,127,.14)}',
'#sky .sk-snack{position:absolute;left:12px;right:12px;bottom:12px;z-index:40;background:var(--inv);color:var(--on-inv);border-radius:6px;',
' padding:14px 16px;font-size:16px;display:flex;align-items:center;gap:12px;box-shadow:0 4px 16px rgba(0,0,0,.3);animation:skUp .2s ease-out;max-width:600px;margin:0 auto}',
'#sky .sk-snack span{flex:1}',
'#sky .sk-snack button{border:0;background:transparent;color:var(--cyan);font-weight:700;font-size:16px;cursor:pointer;padding:4px}',
'#sky.dark .sk-snack button{color:var(--secondary)}',
// busy + splash
'#sky .sk-busy{position:absolute;left:50%;top:50%;width:64px;height:64px;margin:-32px 0 0 -32px;z-index:5;pointer-events:none}',
'#sky .sk-busy img{position:absolute;inset:0;width:100%;height:100%}',
'#sky .sk-busy .rif{animation:skSpin 1.4s linear infinite}',
'#sky .sk-splash{position:absolute;inset:0;z-index:50;background:#FCF9F5;display:flex;flex-direction:column;align-items:center;justify-content:center;transition:opacity .22s}',
'#sky .sk-splash .lg{position:relative;width:min(56vw,260px);height:min(56vw,260px)}',
'#sky .sk-splash .lg img{position:absolute;inset:0;width:100%;height:100%}',
'#sky .sk-splash .rif{animation:skLaunch .9s cubic-bezier(.2,.7,.3,1) both}',
'@keyframes skLaunch{from{transform:rotate(-50deg)}to{transform:rotate(0)}}',
'#sky .sk-splash .w1{margin-top:28px;font-size:min(13vw,64px);font-weight:900;letter-spacing:.02em;color:#01274A;line-height:1}',
'#sky .sk-splash .w2{margin-top:8px;font-size:min(5vw,24px);letter-spacing:.5em;color:#01274A;padding-left:.5em}',
'#sky .sk-splash .w3{position:absolute;bottom:28px;font-size:14px;color:#3A4D66;letter-spacing:.08em}',
'#sky .sk-empty{padding:28px 24px;text-align:center;color:var(--on-var);font-size:17px;white-space:pre-line}',
'#sky .sk-search{display:flex;align-items:center;gap:8px;margin:12px 12px 4px;padding:0 12px;min-height:56px;border-radius:28px;background:var(--s-highest);color:var(--on-var)}',
'#sky .sk-search input{flex:1;border:0;background:transparent;font-size:18px;color:var(--on);outline:none;font-family:inherit;min-height:52px}',
'#sky .sk-bcrow{display:flex;align-items:center;gap:8px;width:100%}',
'#sky .sk-bcrow .sk-num{margin-left:0;flex:1;width:auto;min-width:0}',
'#sky .sk-x{width:44px;height:44px;border:0;background:transparent;color:var(--on-var);border-radius:22px;cursor:pointer;display:flex;align-items:center;justify-content:center}',
'#sky .sk-mono{font-family:ui-monospace,"Roboto Mono",Menlo,monospace}',
'@media (min-width:900px){#sky .sk-res .sk-cell .v{font-size:40px}}'
].join('\n');

// Material icons (the same paths as Skyward's res/drawable vectors)
var IC={
  back:'M20,11H7.83l5.59,-5.59L12,4l-8,8 8,8 1.41,-1.41L7.83,13H20v-2z',
  close:'M19,6.41L17.59,5 12,10.59 6.41,5 5,6.41 10.59,12 5,17.59 6.41,19 12,13.41 17.59,19 19,17.59 13.41,12z',
  target:'M12,2C6.48,2 2,6.48 2,12s4.48,10 10,10 10,-4.48 10,-10S17.52,2 12,2zM13,19.93V17h-2v2.93C7.38,19.48 4.52,16.62 4.07,13H7v-2H4.07C4.52,7.38 7.38,4.52 11,4.07V7h2V4.07c3.62,0.45 6.48,3.31 6.93,6.93H17v2h2.93c-0.45,3.620 -3.31,6.48 -6.93,6.93zM12,10.5c-0.83,0 -1.5,0.67 -1.5,1.5s0.67,1.5 1.5,1.5 1.5,-0.67 1.5,-1.5 -0.67,-1.5 -1.5,-1.5z',
  gear:'M7.5,2C5.9,3.6 5.5,5.6 5.5,8H9.5C9.5,5.6 9.1,3.6 7.5,2ZM5,9H10V20H5ZM4.5,20.6H10.5V22H4.5ZM16.5,2C14.9,3.6 14.5,5.6 14.5,8H18.5C18.5,5.6 18.1,3.6 16.5,2ZM14,9H19V20H14ZM13.5,20.6H19.5V22H13.5Z',
  analysis:'M4,13h3.2v7H4zM10.4,8.5h3.2V20h-3.2zM16.8,4h3.2v16h-3.2zM2,21h20v1.6H2z',
  more:'M3,5.5h18v2.2H3zM3,10.9h18v2.2H3zM3,16.3h18v2.2H3z',
  table:'M3,3h18v18H3V3zM5,5v4h4V5H5zM11,5v4h8V5h-8zM5,11v4h4v-4H5zM11,11v4h8v-4h-8zM5,17v2h4v-2H5zM11,17v2h8v-2h-8z',
  card:'M3,5v14h18V5H3zM19,17H5V7h14v10zM7,9h10v2H7V9zM7,13h7v2H7v-2z',
  wrench:'M22.7,19l-9.1,-9.1c0.9,-2.3 0.4,-5 -1.5,-6.9 -2,-2 -5,-2.4 -7.4,-1.3L9,6 6,9 1.6,4.7C0.4,7.1 0.9,10.1 2.9,12.1c1.9,1.9 4.6,2.4 6.9,1.5l9.1,9.1c0.4,0.4 1,0.4 1.4,0l2.3,-2.3c0.5,-0.4 0.5,-1.1 0.1,-1.4z',
  chevron:'M10,6L8.59,7.41 13.17,12l-4.58,4.59L10,18l6,-6z',
  settings:'M19.14,12.94c0.04,-0.3 0.06,-0.61 0.06,-0.94c0,-0.32 -0.02,-0.64 -0.07,-0.94l2.03,-1.58c0.18,-0.14 0.23,-0.41 0.12,-0.61l-1.92,-3.32c-0.12,-0.22 -0.37,-0.29 -0.59,-0.22l-2.39,0.96c-0.5,-0.38 -1.03,-0.7 -1.62,-0.94L14.4,2.81c-0.04,-0.24 -0.24,-0.41 -0.48,-0.41h-3.84c-0.24,0 -0.43,0.17 -0.47,0.41L9.25,5.35C8.66,5.59 8.12,5.92 7.63,6.29L5.24,5.33c-0.22,-0.08 -0.47,0 -0.59,0.22L2.74,8.87C2.62,9.08 2.66,9.34 2.86,9.48l2.03,1.58C4.84,11.36 4.8,11.69 4.8,12s0.02,0.64 0.07,0.94l-2.03,1.58c-0.18,0.14 -0.23,0.41 -0.12,0.61l1.92,3.32c0.12,0.22 0.37,0.29 0.59,0.22l2.39,-0.96c0.5,0.38 1.03,0.7 1.62,0.94l0.36,2.54c0.05,0.24 0.24,0.41 0.48,0.41h3.84c0.24,0 0.44,-0.17 0.47,-0.41l0.36,-2.54c0.59,-0.24 1.13,-0.56 1.62,-0.94l2.39,0.96c0.22,0.08 0.47,0 0.59,-0.22l1.92,-3.32c0.12,-0.22 0.07,-0.47 -0.12,-0.61L19.14,12.94zM12,15.6c-1.98,0 -3.6,-1.62 -3.6,-3.6s1.62,-3.6 3.6,-3.6s3.6,1.62 3.6,3.6S13.98,15.6 12,15.6z',
  check:'M9,16.17L4.83,12l-1.42,1.41L9,19 21,7l-1.41,-1.41z',
  dots:'M12,8c1.1,0 2,-0.9 2,-2s-0.9,-2 -2,-2 -2,0.9 -2,2 0.9,2 2,2zM12,10c-1.1,0 -2,0.9 -2,2s0.9,2 2,2 2,-0.9 2,-2 -0.9,-2 -2,-2zM12,16c-1.1,0 -2,0.9 -2,2s0.9,2 2,2 2,-0.9 2,-2 -0.9,-2 -2,-2z',
  add:'M19,13h-6v6h-2v-6H5v-2h6V5h2v6h6v2z',
  share:'M18,16.08c-0.76,0 -1.44,0.3 -1.96,0.77L8.91,12.7c0.05,-0.23 0.09,-0.46 0.09,-0.7s-0.04,-0.47 -0.09,-0.7l7.05,-4.11c0.54,0.5 1.25,0.81 2.04,0.81 1.66,0 3,-1.34 3,-3s-1.34,-3 -3,-3 -3,1.34 -3,3c0,0.24 0.04,0.47 0.09,0.7L8.04,9.81C7.5,9.31 6.79,9 6,9c-1.66,0 -3,1.34 -3,3s1.34,3 3,3c0.79,0 1.5,-0.31 2.04,-0.81l7.12,4.16c-0.05,0.21 -0.08,0.43 -0.08,0.65 0,1.61 1.31,2.92 2.92,2.92 1.61,0 2.92,-1.31 2.92,-2.92s-1.31,-2.92 -2.92,-2.92z',
  search:'M15.5,14h-0.79l-0.28,-0.27C15.41,12.59 16,11.11 16,9.5 16,5.91 13.09,3 9.5,3S3,5.91 3,9.5 5.91,16 9.5,16c1.61,0 3.09,-0.59 4.23,-1.57l0.27,0.28v0.79l5,4.99L20.49,19l-4.99,-5zM9.5,14C7.01,14 5,11.99 5,9.5S7.01,5 9.5,5 14,7.01 14,9.5 11.99,14 9.5,14z',
  wind:'M4,10h11.5c1.9,0 3.5,-1.6 3.5,-3.5S17.4,3 15.5,3c-1.6,0 -2.9,1 -3.3,2.5l1.9,0.6c0.2,-0.6 0.8,-1.1 1.4,-1.1 0.8,0 1.5,0.7 1.5,1.5S16.3,8 15.5,8H4V10zM4,12v2h14.5c0.8,0 1.5,0.7 1.5,1.5S19.3,17 18.5,17c-0.6,0 -1.2,-0.4 -1.4,-1l-1.9,0.6c0.5,1.4 1.8,2.4 3.3,2.4 1.9,0 3.5,-1.6 3.5,-3.5S20.4,12 18.5,12H4z',
  sync:'M12,4V1L8,5l4,4V6c3.31,0 6,2.69 6,6 0,1.01 -0.25,1.97 -0.7,2.8l1.46,1.46C19.54,15.03 20,13.57 20,12c0,-4.42 -3.58,-8 -8,-8zM12,18c-3.31,0 -6,-2.69 -6,-6 0,-1.01 0.25,-1.97 0.7,-2.8L5.24,7.74C4.46,8.97 4,10.43 4,12c0,4.42 3.58,8 8,8v3l4,-4 -4,-4v3z',
  upload:'M9,16h6v-6h4l-7,-7 -7,7h4zM5,18h14v2H5z',
  download:'M19,9h-4V3H9v6H5l7,7 7,-7zM5,18v2h14v-2H5z',
  info:'M11,7h2v2h-2zM11,11h2v6h-2zM12,2C6.48,2 2,6.48 2,12s4.48,10 10,10 10,-4.48 10,-10S17.52,2 12,2zM12,20c-4.41,0 -8,-3.59 -8,-8s3.59,-8 8,-8 8,3.59 8,8 -3.59,8 -8,8z',
  calc:'M19,3H5C3.9,3 3,3.9 3,5v14c0,1.1 0.9,2 2,2h14c1.1,0 2,-0.9 2,-2V5C21,3.9 20.1,3 19,3zM13.03,7.06L14.09,6l1.41,1.41L16.91,6l1.06,1.06 -1.41,1.41 1.41,1.41 -1.06,1.06 -1.41,-1.4 -1.41,1.41 -1.06,-1.06 1.41,-1.41 -1.41,-1.42zM6.25,7.72h5v1.5h-5V7.72zM11.5,16h-2v2H8v-2H6v-1.5h2v-2h1.5v2h2V16zM18,17.25h-5v-1.5h5V17.25zM18,14.75h-5v-1.5h5V14.75z'
};
function icon(name,cls){
  var s=doc.createElementNS('http://www.w3.org/2000/svg','svg');
  s.setAttribute('viewBox','0 0 24 24');s.setAttribute('class','sk-ic'+(cls?' '+cls:''));s.setAttribute('aria-hidden','true');
  var p=doc.createElementNS('http://www.w3.org/2000/svg','path');p.setAttribute('d',IC[name]||'');s.appendChild(p);
  return s;
}
// Skyward's logo: the bore (sun + cloud) stays still, the rifling turns.
var LOGO={bore:'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAKAAAACgCAMAAAC8EZcfAAAASFBMVEUAs/0D0v3l+f4S3P0D1P2c6/xf4Pz+/v4B0/z//v4Af//+/v4AAP9xuP9q//8AAAAA0f79/f4A//8Azvz+/v4C1fwB1/4B1P3fdswJAAAAGHRSTlMDFuX9if7/WmcwAogBAwIA/v4C/QTMVDOTr3Z1AAADoElEQVR42u3b63ajIBAAYMB7bAty0fd/00WMG9tNqzOAcU9nTn80bVK+MwOCSJm8eDACEpCABCQgAQlIQAISkIAEJCABCUhAAhKQgAQkIAEJSMDLA/VNM+2D+W8uB5xZn1576mWAb3cL59MYYuL8rn67ADDo9DSWzhm1hnPlyPX621cCZ4CeykKpYqGZ8LUoy0nHE1lccX1hW6ceqL+x/MC1vthxhWZxn+Wl+j6M8XkteXQj+Oqy9lnyviayZTF1RgN9k6NTu+GzqNwY3n4u0Kev3MneJoslQwsZ9mOTC9lRx4huwreE9KnjvhATuinUh9qlvBDjiG0rwgcIgxUyzPhoobw1h/oMoA79DwP0/VDnB2rJZ99pQijQL/TcdkUAC8eZzgxksiyw+fNLnhLRILgDYnW4IsOAWnMXB3QcuHBg51xhHkVuZUbgTXMVHVzfsgG1LCMT6D9ewlLIYD1QqRQp1JmAYQ6OjxbYKOAWibkUQMcgt1EM8tZJJYkJ1ipoiKSIAjRMIEDt0mTQ6SzAiGVWzHwHAY4qUYxZgPM6JhGwBDV7eheEdUJ2vMI8HZAfr/Fh4E1ylSy4vGXI4AS8VU8zjNnxN47KpBvGLEMGkwL1bwUmIZqrZzAP8D8YJFM64PQ7L9QXn+ouv1iYl1vwTmj87Gi+zpBFpuUWfBgbVVeVEFVVb40m04JVQ2/qjKnFcA9Rm+xLfngnrIZNVI8KZ7ppmm87C0h5xfApxJpX3wU/XnzjbuZx8cUXchierRRcvr9068MUdV3V1fBP1MtiI9vWx7x5VBzgPaHdI1xximybR8e234rh+6jqujAZt98ODBPzk2+pc8YNzANbwKbaAw5Wf2QDzpvohXk+asPIVbsJHBppcz+GeDahiTChKVPv+YS1OYFP57vthLZb4Q6YQOSjsO2oEAMgeqgP/TAR5wMXGPU4dj2uEMYFyAcvMOaB9nKgIhwrqlWd24c7EjDO6asGaDQIH/Kcw6jO8qGPpZzlQ562eZcd1NfhfNgDS14Iu75gfejDZVbaHnB9tlgf/nieb7A5mETRSLQv4oCjnxNsc2h0WGnRvqjTm77V/Tr3NiJ9kcCQGPtToUVjZUz6ZPQx5UDs+qdG0XfRvAQHvQPAdk0vNkoh+qazMp6X5Kj8uoSyXdeE6Dr76RevBt4t9qfXLweuqDXS/VX6jxwCEpCABCQgAQlIQAISkIAEJCABCUhAAhKQgAQkIAEJ+FuBfwCj6ev40EjzlAAAAABJRU5ErkJggg==',
  rifLight:'data:image/png;base64,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',
  rifNavy:'data:image/png;base64,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'};


// ============================================================================
// Small helpers
// ============================================================================
function el(tag,cls,text){var e=doc.createElement(tag);if(cls)e.className=cls;if(text!==undefined&&text!==null)e.textContent=text;return e;}
function add(p){for(var i=1;i<arguments.length;i++){var c=arguments[i];if(c)p.appendChild(c);}return p;}
function on(e,ev,f){e.addEventListener(ev,f);return e;}
function ls(k,d){try{var v=root.localStorage.getItem(k);return v===null?d:v;}catch(e){return d;}}
function lsSet(k,v){try{root.localStorage.setItem(k,v);}catch(e){}}
function lsJson(k,d){try{var v=JSON.parse(root.localStorage.getItem(k));return v===null||v===undefined?d:v;}catch(e){return d;}}
function fin(v,d){v=parseFloat(v);return isFinite(v)?v:d;}
function nrm(deg){return ((deg%360)+360)%360;}
var MOA_PER_MIL=3.43774677;

var Num={
  trim:function(v,d){if(!isFinite(v))return '—';var f=Math.pow(10,d),r=Math.round(v*f)/f;if(r===0)r=0;
    var s=r.toFixed(Math.max(0,d));return s.indexOf('.')>=0?s.replace(/0+$/,'').replace(/\.$/,''):s;},
  fixed:function(v,d){if(!isFinite(v))return '—';var f=Math.pow(10,d),r=Math.round(v*f)/f;if(r===0)r=0;return r.toFixed(Math.max(0,d));},
  parse:function(t){if(t===null||t===undefined)return null;t=(''+t).trim().replace(',','.');if(t===''||t==='-'||t==='+'||t==='.')return null;
    var v=Number(t);return isFinite(v)?v:null;},
  decimalsOf:function(v){for(var d=0;d<=4;d++){var f=Math.pow(10,d);if(Math.abs(v*f-Math.round(v*f))<1e-6)return d;}return 4;}
};

// ---- settings (Skyward AppSettings) ----
var SET_DEF={distanceUnit:'yd',atmosphereUnits:'imperial',otherUnits:'imperial',outputPrecision:'tenth',usePlusMinus:false,
  solutionStepYd:5,tableStepYd:50,tableMaxYd:2000,tableSortDescending:false,useDensityAltitude:false,theme:'system',motion:'full'};
function S(){var s=lsJson('skySettings',{});var o={};for(var k in SET_DEF)o[k]=(s&&s[k]!==undefined)?s[k]:SET_DEF[k];return o;}
function saveS(s){lsSet('skySettings',JSON.stringify(s));applyTheme();if(root.wlTheme)try{root.wlTheme();}catch(e){}}

// ---- units (Skyward ui/calc/Units.kt) ----
var Q={DISTANCE:'distance',ALTITUDE:'altitude',PRESSURE:'pressure',TEMPERATURE:'temperature',HUMIDITY:'humidity',LENGTH:'length',
  VELOCITY:'velocity',ENERGY:'energy',SPEED:'speed',VEL_PER_DEG:'velPerDeg',ANGLE:'angle',BULLET_LENGTH:'bulletLength',WEIGHT:'weight',PLAIN:'plain'};
var Units={
  metric:function(q,s){s=s||S();
    if(q===Q.DISTANCE)return s.distanceUnit==='m';
    if(q===Q.ALTITUDE||q===Q.PRESSURE||q===Q.TEMPERATURE)return s.atmosphereUnits==='metric';
    if(q===Q.LENGTH||q===Q.VELOCITY||q===Q.ENERGY||q===Q.SPEED||q===Q.VEL_PER_DEG)return s.otherUnits==='metric';
    return false;},
  toDisplay:function(q,v,s){if(v===null||v===undefined||!Units.metric(q,s))return v;
    switch(q){case Q.DISTANCE:return v*0.9144;case Q.ALTITUDE:return v*0.3048;case Q.PRESSURE:return v*33.8638866667;
      case Q.TEMPERATURE:return (v-32)*5/9;case Q.LENGTH:return v*2.54;case Q.VELOCITY:return v*0.3048;case Q.ENERGY:return v*1.3558179483;
      case Q.SPEED:return v*0.44704;case Q.VEL_PER_DEG:return v*0.3048*1.8;}return v;},
  fromDisplay:function(q,v,s){if(v===null||v===undefined||!Units.metric(q,s))return v;
    switch(q){case Q.DISTANCE:return v/0.9144;case Q.ALTITUDE:return v/0.3048;case Q.PRESSURE:return v/33.8638866667;
      case Q.TEMPERATURE:return v*9/5+32;case Q.LENGTH:return v/2.54;case Q.VELOCITY:return v/0.3048;case Q.ENERGY:return v/1.3558179483;
      case Q.SPEED:return v/0.44704;case Q.VEL_PER_DEG:return v/(0.3048*1.8);}return v;},
  label:function(q,s){var m=Units.metric(q,s);
    return {distance:m?'m':'yd',altitude:m?'m':'ft',pressure:m?'hPa':'inHg',temperature:m?'°C':'°F',humidity:'%',length:m?'cm':'in',
      velocity:m?'m/s':'fps',energy:m?'J':'ft-lb',speed:m?'m/s':'mph',velPerDeg:m?'m/s/°C':'fps/°F',angle:'°',bulletLength:'in',weight:'gr',plain:''}[q];},
  decimals:function(q,s){var m=Units.metric(q,s);
    return {distance:0,altitude:0,pressure:m?1:2,temperature:m?1:0,humidity:0,length:2,velocity:m?1:0,energy:0,speed:1,velPerDeg:2,angle:1,bulletLength:3,weight:1,plain:3}[q];},
  text:function(q,v,d){var s=S();return Num.trim(Units.toDisplay(q,v,s),d===undefined?Units.decimals(q,s):d)+' '+Units.label(q,s);}
};
function dist(yd){return Units.text(Q.DISTANCE,yd,0);}
function speedTxt(mph){return Units.text(Q.SPEED,mph,1);}

// ---- scope adjustments (Skyward CalcLogic.kt Adjust) ----
function milTo(unit,mil){return unit==='MOA'?mil*MOA_PER_MIL:(unit==='IPHY'?mil*3.6:mil);}
var Adjust={
  unitLabel:function(u){return u==='MOA'?'MOA':(u==='IPHY'?'IPHY':'MIL');},
  dial:function(mil,unit,corr){return milTo(unit,mil)*(corr>0?corr:1);},
  prefix:function(v,axis,pm){if(v===0)return '';if(pm)return v>0?'+':'-';return axis==='E'?(v>0?'U':'D'):(v>0?'R':'L');},
  text:function(value,axis,prec,click,pm){
    if(!isFinite(value))return '—';
    var r,d;
    if(prec==='click'&&click>0){r=Math.round(value/click)*click;d=Num.decimalsOf(click);}
    else if(prec==='hundredth'){r=Math.round(value*100)/100;d=2;}
    else{r=Math.round(value*10)/10;d=1;}
    if(Math.abs(r)<1e-9)return '0';
    return Adjust.prefix(r,axis,pm)+Num.fixed(Math.abs(r),d);},
  clicks:function(mil,unit,click,corr){return milTo(unit,mil)*(corr>0?corr:1)/(click>0?click:1);},
  clicksText:function(c,axis,pm){if(!isFinite(c))return '—';var n=Math.round(c);if(n===0)return '0';return Adjust.prefix(n,axis,pm)+Math.abs(n);}
};
var WindText={
  clock:function(deg){var d=nrm(deg),h=Math.round(d/30);if(h===0||h===12)h=12;
    var side=(d<15||d>345)?'head':((d>=165&&d<=195)?'tail':(d<180?'from right':'from left'));return h+" o'clock · "+side;},
  hour:function(deg){return WindText.clock(deg).split(' · ')[0];},
  side:function(deg){var s=WindText.clock(deg).split(' · ')[1];
    return s==='head'?'head wind, in your face':(s==='tail'?'tail wind, from behind':(s==='from right'?'from your right':'from your left'));},
  direction:function(t){var c=(t||'').charAt(0);return c==='U'?'Up · ':(c==='D'?'Down · ':(c==='R'?'Right · ':(c==='L'?'Left · ':'')));}
};

// ---- density altitude (Skyward CalcLogic.kt) ----
var DA={
  of:function(a){try{var rho=new SkyEngine.Air(a).density;return ((1-Math.pow(rho/1.225,1/4.2559))/2.25577e-5)/0.3048;}catch(e){return NaN;}},
  atmosphere:function(daFt,tempF){var hM=daFt*0.3048,rho=1.225*Math.pow(Math.max(0.05,1-2.25577e-5*hM),4.2559);
    var tK=(tempF-32)*5/9+273.15,pPa=rho*287.058*tK;return {altitudeFt:daFt,pressureInHg:pPa/3386.389,pressureIsAbsolute:true,temperatureF:tempF,humidityPct:0};}
};

// ============================================================================
// Bridge to the Wind Lab page. The Skyward screens read and write the SAME
// state the rest of Coriolis uses (sol_* fields, live toggles, activeLoadId,
// firing azimuth), so WIND HOLD, the aimbot, the log, auto weights and the
// sandbox always work from what this screen shows.
// ============================================================================
function G(){return root.Gear;}
function getv(k,d){return root.getv?root.getv(k,d):ls('sol_'+k,d);}
function setv(k,v){if(root.setv)root.setv(k,v);else lsSet('sol_'+k,v);}
function liveOn(key){return root.liveOn?root.liveOn(key):ls(key,'1')==='1';}
function setLive(key,onv){lsSet(key,onv?'1':'0');}
function opt(k,d){return ls('opt_'+k,d);}
function setOpt(k,v){lsSet('opt_'+k,v);}
function wl(){return root.data||null;}
function fireAz(){return fin(root.fireAz,0);}
function setFireAz(v){v=Math.round(nrm(v));try{root.fireAz=v;}catch(e){}lsSet('fireAz',v);
  var a=doc.getElementById('az');if(a)a.value=v;if(root.saveAz)try{root.saveAz();}catch(e){}if(root.drawWheel)try{root.drawWheel();}catch(e){}}
function liveWind(){
  var d=wl();if(!d||!root.avgWind)return {ok:false,speed:0,rel:0,count:0};
  var w=root.avgWind(),a=root.weightedAvgWind?root.weightedAvgWind():{count:w.ok?1:0};
  return {ok:w.ok,speed:w.speed,dir:w.dir,rel:root.relAngle?root.relAngle(w.dir):nrm(w.dir-fireAz()),count:a.count||0,bad:a.bad||0};
}
function baseOk(){var d=wl();return !!(d&&d.base);}

// Flat profile + shot, like solverParams() in WebUI.h but it works with no base station data.
function params(){
  var p={};
  (root.LOAD||[]).forEach(function(f){p[f.k]=getv(f.k,f.d);});
  if(!root.LOAD)p.range=getv('range',500);
  var d=wl(),w=liveWind();
  p.tempF=(liveOn('live_temp')&&baseOk())?d.base.tempF:getv('tempF',59);
  p.pres=(liveOn('live_pres')&&baseOk())?d.base.presInHg:getv('pres',29.92);
  p.windmph=(liveOn('live_wind')&&w.ok)?w.speed:getv('windmph',10);
  p.windrel=(liveOn('live_wind')&&w.ok)?w.rel:getv('windrel',90);
  p.lat=(liveOn('live_lat')&&baseOk()&&d.base.gps)?d.base.lat:getv('lat',45);
  p.spin=opt('spin','1');p.jump=opt('jump','1');p.earth=opt('earth','0');p.spinModel=opt('spinModel','LITZ_CLASSIC');
  p.az=fireAz();p.bulletId=getv('bulletId','');p.loadId=activeId();
  p.look=getv('look',0);p.cant=getv('cant',0);p.tgtMph=getv('tgtMph',0);p.wind2=getv('wind2',0);
  p.hum=getv('hum',0);p.alt=getv('alt',0);p.localG=opt('localG','0');
  p.presAbs=liveOn('live_pres')?'1':getv('presAbs','1');
  return p;
}
function shotOf(p){
  return {distanceYd:fin(p.range,0),windSpeed1Mph:fin(p.windmph,0),windAngleDeg:fin(p.windrel,90),windSpeed2Mph:fin(p.wind2,0),
    lookAngleDeg:fin(p.look,0),cantDeg:fin(p.cant,0),targetSpeedMph:fin(p.tgtMph,0),
    spinDrift:(''+p.spin)!=='0',aeroJump:(''+p.jump)!=='0',spinDriftModel:p.spinModel||'LITZ_CLASSIC',
    coriolis:(''+p.earth)==='1',latitudeDeg:fin(p.lat,0),azimuthDeg:fin(p.az,0),localGravity:(''+p.localG)==='1',
    atmosphere:G().atmoOf(p)};
}
// Group offset (where the zero group landed) in mil, added to every hold
// exactly like the Wind Lab solver and WIND HOLD do.
function offsets(){
  var em=fin(getv('zeroElevMag',0),0),ed=getv('zeroElevDir','high'),wm=fin(getv('zeroWindMag',0),0),wd=getv('zeroWindDir','right');
  return {e:((ed==='high')?-em:em)/MOA_PER_MIL,w:((wd==='right')?-wm:wm)/MOA_PER_MIL};
}
// One full solution for the current shot. over = fields to change (range, ...).
function solveNow(over,optn){
  var p=params();if(over)for(var k in over)p[k]=over[k];
  var g=G().engineGear(p);
  var sol=SkyEngine.solve(g.rifle,g.load,g.bullet,shotOf(p),optn||{maxRangeYd:fin(p.range,0),stepYd:Math.max(1,fin(p.range,0))});
  var off=offsets();
  sol.rows.forEach(function(r){if(r.rangeYd>0){r.comeUpMil+=off.e;r.windageMil+=off.w;if(r.windage2Mil!==null)r.windage2Mil+=off.w;}});
  sol.p=p;sol.gear=g;
  return sol;
}

// ---- selection: Skyward selects a firearm and an ammo; Coriolis keeps the ammo (load) id ----
function activeId(){return root.activeLoadId?root.activeLoadId():ls('activeLoadId','');}
function rifles(){var g=G();if(!g)return [];return g.rifles().slice().sort(function(a,b){var x=(a.name||'').toLowerCase(),y=(b.name||'').toLowerCase();return x<y?-1:x>y?1:0;});}
function loadsOf(rid){var g=G();if(!g)return [];return g.lib().loads.filter(function(l){return l.rifleId===rid;}).sort(function(a,b){var x=(a.name||'').toLowerCase(),y=(b.name||'').toLowerCase();return x<y?-1:x>y?1:0;});}
function sel(){
  var g=G(),lid=activeId(),l=lid&&g?g.load(lid):null;
  var r=l?g.rifle(l.rifleId):null;
  if(!r){var rid=ls('skyRifleId','');r=(g&&rid)?g.rifle(rid):null;}
  if(!r){var rs=rifles();r=rs[0]||null;}
  if(r&&(!l||l.rifleId!==r.id)){var ls2=loadsOf(r.id);l=ls2[0]||null;if(l)useLoad(l.id,true);}
  var b=l&&g?g.bullet(l.bulletId):null;
  return {rifle:r,load:l,bullet:b,complete:!!(r&&l&&b)};
}
// Make a load the one every Coriolis feature solves with.
function useLoad(id,quiet){
  var g=G();var l=id&&g.load(id);
  if(!l){if(root.setActiveId)root.setActiveId('');else lsSet('activeLoadId','');return;}
  lsSet('skyRifleId',l.rifleId);
  var f=g.flat(id);f.loadId=id;
  if(root.applyProfile)root.applyProfile(f);else for(var k in f)setv(k,f[k]);
  if(root.setActiveId)root.setActiveId(id);else lsSet('activeLoadId',id);
  if(!quiet)recentUse(l.rifleId,id);
}
function useRifle(rid){
  lsSet('skyRifleId',rid);
  var cur=activeId(),l=cur&&G().load(cur);
  if(l&&l.rifleId===rid)return;
  var rec=recentOrder(loadsOf(rid).map(function(x){return x.id;}),'skyRecentLoads');
  if(rec.length)useLoad(rec[0]);else{if(root.setActiveId)root.setActiveId('');else lsSet('activeLoadId','');recentUse(rid,null);}
}
function recentUse(rid,lid){
  function bump(key,id){if(!id)return;var a=lsJson(key,[]).filter(function(x){return x!==id;});a.unshift(id);lsSet(key,JSON.stringify(a.slice(0,30)));}
  bump('skyRecentRifles',rid);bump('skyRecentLoads',lid);
}
function recentOrder(ids,key){var rec=lsJson(key,[]);return ids.slice().sort(function(a,b){var i=rec.indexOf(a),j=rec.indexOf(b);if(i<0)i=1e9;if(j<0)j=1e9;return i-j;});}
// Re-apply the active load's fields after an edit here (no "updated on another device" note).
function refreshActive(){var id=activeId();if(id&&G().load(id)){root.__skyQuiet=true;try{useLoad(id,true);}finally{root.__skyQuiet=false;}}}

// Text helpers that mirror Skyward's
function rifleSub(r){
  return '1:'+Num.trim(fin(r.twistIn,0),2)+'" '+(r.twistDirection==='LEFT'?'L':'R')+' · sight '+Num.trim(fin(r.sightHeightIn,0),2)+'" · '+
    Adjust.unitLabel(r.elevationUnit||'MIL')+' '+Num.trim(fin(r.elevationClick,0.1),3)+'/click';
}
function bcSummary(l,b){
  if(!b)return 'bullet missing';
  var w=Num.trim(fin(b.weightGr,0),1)+'gr',model=l.dragModelOverride||b.dragModel||'G7';
  if(model==='CUSTOM')return w+' drag curve';
  if(l.bcSeason==='auto')return w+' G7 seasonal (by temperature)';
  if(l.bcSeason==='cold'||l.bcSeason==='mild'||l.bcSeason==='hot')return w+' G7 '+l.bcSeason;
  var rows=(l.bcPointsOverride&&l.bcPointsOverride.length)?l.bcPointsOverride:(b.bcPoints||[]);
  if(!rows.length)return w+' '+model+' (no BC)';
  if(rows.length===1)return w+' '+model+' '+Num.trim(rows[0].bc,3);
  return w+' '+model+' ('+rows.length+' BCs)';
}
function loadFlags(l){var f=[];if(fin(l.muzzleVelocityFps,0)<300)f.push('Muzzle velocity needed');if(l.zeroConfirmed===false)f.push('Zero not confirmed');return f.length?'⚠ '+f.join(' · '):null;}
function loadSub(l,b){var mv=fin(l.muzzleVelocityFps,0)>=300?Units.text(Q.VELOCITY,l.muzzleVelocityFps,0):null;return [loadFlags(l),bcSummary(l,b),mv].filter(Boolean).join(' · ');}
function bulletTitle(b){return (b.maker&&(b.name||'').toLowerCase().indexOf(b.maker.toLowerCase())!==0)?(b.maker+' '+(b.name||'')):(b.name||'Bullet');}
function guard(r,l,b){
  if(!(fin(l.muzzleVelocityFps,0)>0))return "Ammo '"+l.name+"' has no muzzle velocity. Edit the ammo and enter one.";
  if(!(fin(l.zeroRangeYd,0)>0))return "Ammo '"+l.name+"' has no zero range. Edit the ammo and enter one.";
  if(!(fin(b.diameterIn,0)>0&&fin(b.weightGr,0)>0))return "The bullet of '"+l.name+"' has no diameter or weight. Edit the ammo.";
  var model=l.dragModelOverride||b.dragModel,rows=(l.bcPointsOverride&&l.bcPointsOverride.length)?l.bcPointsOverride:(b.bcPoints||[]);
  if(model==='CUSTOM'?!((b.dragCurve||[]).length>=2):!rows.some(function(x){return x.bc>0;}))return "Ammo '"+l.name+"' has no ballistic coefficient or drag curve. Edit the ammo and enter a BC.";
  if(!(fin(r.twistIn,0)>0))return "Firearm '"+r.name+"' has no barrel twist. Edit the firearm.";
  return null;
}
function rifleUnits(r){r=r||{};return {eu:r.elevationUnit||'MIL',wu:r.windageUnit||'MIL',ec:fin(r.elevationClick,0.1),wc:fin(r.windageClick,0.1),
  ecorr:fin(r.elevationCorrection,1),wcorr:fin(r.windageCorrection,1)};}

// ============================================================================
// Form builder (Skyward ui/calc/Form.kt)
// ============================================================================
function Form(box){this.box=box;}
Form.prototype.header=function(title,actionLabel,action){
  var row=add(el('div','sk-h'),el('div','bar'),el('div','t',title));
  if(actionLabel&&action)row.appendChild(on(el('button','sk-tbtn',actionLabel),'click',action));
  this.box.appendChild(row);return row;};
Form.prototype.note=function(t){var n=el('div','sk-note',t);this.box.appendChild(n);return n;};
Form.prototype.add=function(v){this.box.appendChild(v);return v;};
Form.prototype.group=function(fn){var g=el('div','sk-group');this.box.appendChild(g);var G2=new Group(g);if(fn)fn(G2);return G2;};
Form.prototype.button=function(text,fn,outlined){var w=el('div','sk-bwrap');var b=on(el('button',outlined?'sk-obtn':'sk-btn',text),'click',fn);if(outlined)b.style.width='100%';w.appendChild(b);this.box.appendChild(w);return b;};
Form.prototype.foldable=function(key,title,summary,block){
  this.box.appendChild(el('div','sk-gap'));
  var open=ls('skyOpen_'+key,'0')==='1';
  var g=this.group();
  var row=g.row(false,true);row.classList.add('sk-fold');
  var col=el('div','sk-lab');col.appendChild(el('div','l b',title));
  var sum=el('div','s sum',summary());col.appendChild(sum);row.appendChild(col);
  var hint=el('div','hint');row.appendChild(hint);
  var ch=icon('chevron','sk-chev');row.appendChild(ch);
  var body=el('div','sk-body');g.box.appendChild(body);
  var inner=new Group(body,true);block(inner);
  function show(o,anim){
    hint.textContent=o?'Close':'Open';
    ch.style.transform='rotate('+(o?270:90)+'deg)';
    sum.style.display=o?'none':'';if(!o)sum.textContent=summary();
    if(anim&&!still()){
      body.style.display='';var h=body.scrollHeight;body.style.height=(o?0:h)+'px';body.style.transition='height .22s ease';
      requestAnimationFrame(function(){body.style.height=(o?h:0)+'px';});
      setTimeout(function(){body.style.transition='';body.style.height='';body.style.display=o?'':'none';},240);
    }else body.style.display=o?'':'none';
  }
  show(open,false);
  on(row,'click',function(){open=!open;lsSet('skyOpen_'+key,open?'1':'0');blurAll();show(open,true);});
  return g;
};

function Group(box,dividerFirst){this.box=box;this.count=dividerFirst?1:0;}
Group.prototype.row=function(vertical,tap){
  if(this.count++>0)this.box.appendChild(el('div','sk-div'));
  var r=el('div','sk-row'+(vertical?' v':'')+(tap?' tap':''));this.box.appendChild(r);return r;};
function label(parent,text,sub,bold){
  var col=el('div','sk-lab');col.appendChild(el('div','l'+(bold?' b':''),text));
  var s=null;if(sub!==null&&sub!==undefined){s=el('div','s',sub);col.appendChild(s);}
  parent.appendChild(col);col.sub=s;return col;
}
function numInput(signed,hint,big){
  var e=el('input','sk-num'+(big?' big':''));e.type='text';e.inputMode='decimal';e.autocomplete='off';e.spellcheck=false;
  if(hint)e.placeholder=hint;
  on(e,'focus',function(){setTimeout(function(){try{e.select();}catch(x){}},0);});
  on(e,'keydown',function(ev){if(ev.key==='Enter'){e.blur();}});
  return e;
}
// A number row. q = quantity (unit conversion), value in engine units.
Group.prototype.number=function(lab,q,value,onChange,o){
  o=o||{};var s=S();
  var row=this.row(!!o.action);
  var top=o.action?add(el('div','sk-line')):row;if(o.action)row.appendChild(top);
  var L=label(top,lab,o.sub);
  var dec=o.decimals!==undefined?o.decimals:Units.decimals(q,s);
  var e=numInput(o.signed,o.hint||'');
  var nr={edit:e,row:row,label:L,muted:false,
    set:function(v){nr.muted=true;e.value=(v===null||v===undefined||!isFinite(v))?'':Num.trim(Units.toDisplay(q,v,s),dec);nr.muted=false;},
    setEnabled:function(b){e.disabled=!b;},
    sub:function(t){if(L.sub)L.sub.textContent=t;}};
  nr.set(value);
  if(o.signed){
    var pm=on(el('button','sk-pm','+/−'),'click',function(ev){ev.stopPropagation();var t=e.value.trim();e.value=t.charAt(0)==='-'?t.slice(1):(t?'-'+t:'-');fire();});
    pm.setAttribute('aria-label','Change sign of '+lab);top.appendChild(pm);
  }
  if(o.width)e.style.width=o.width;
  top.appendChild(e);if(o.signed)e.style.marginLeft='0';
  var u=Units.label(q,s);if(o.unit!==undefined)u=o.unit;if(u)top.appendChild(el('div','sk-unit',u));
  function fire(){if(nr.muted)return;var v=Num.parse(e.value);onChange(v===null?null:Units.fromDisplay(q,v,s));}
  on(e,'input',fire);
  if(o.action){var under=el('div','sk-under');o.action.forEach(function(a){under.appendChild(on(el('button','sk-pill',a[0]),'click',function(ev){ev.stopPropagation();a[1]();}));});row.appendChild(under);}
  on(row,'click',function(ev){if(ev.target===row||ev.target.parentNode===L||ev.target===L)e.focus();});
  return nr;
};
Group.prototype.stepper=function(lab,q,value,onChange,onStep,o){
  o=o||{};var s=S();
  var row=this.row(true);var top=el('div','sk-line');row.appendChild(top);
  var L=label(top,lab,o.sub,true);
  if(o.badge)L.firstChild.appendChild(o.badge);
  var dec=o.decimals!==undefined?o.decimals:Units.decimals(q,s);
  var e=numInput(false,'',true);
  var nr={edit:e,row:row,label:L,muted:false,
    set:function(v){nr.muted=true;e.value=(v===null||v===undefined||!isFinite(v))?'':Num.trim(Units.toDisplay(q,v,s),dec);nr.muted=false;},
    setEnabled:function(b){e.disabled=!b;minus.disabled=!b;plus.disabled=!b;}};
  nr.set(value);
  var ctr=el('div','sk-steprow');
  var minus=stepBtn('−','Decrease '+lab,function(){e.blur();onStep(-1);});
  var plus=stepBtn('+','Increase '+lab,function(){e.blur();onStep(1);});
  var mid=el('div','mid');mid.appendChild(e);var u=o.unit!==undefined?o.unit:Units.label(q,s);if(u)mid.appendChild(el('div','sk-unit big',u));
  add(ctr,minus,mid,plus);row.appendChild(ctr);
  on(e,'input',function(){if(nr.muted)return;var v=Num.parse(e.value);onChange(v===null?null:Units.fromDisplay(q,v,s));});
  return nr;
};
function stepBtn(glyph,desc,fn){var b=el('button','sk-step',glyph);b.setAttribute('aria-label',desc);on(b,'click',fn);return b;}
Group.prototype.text=function(lab,value,onChange,o){
  o=o||{};var row=this.row(true);
  if(lab){var l=el('div','sk-line');label(l,lab);row.appendChild(l);}
  var e=el(o.multiline?'textarea':'input','sk-txt');if(!o.multiline)e.type='text';e.value=value||'';e.placeholder=o.hint||'';
  if(!lab)e.style.marginTop='0';
  on(e,'input',function(){onChange(e.value);});row.appendChild(e);return e;
};
Group.prototype.sw=function(lab,checked,sub,onChange){
  var row=this.row(false,true);var L=label(row,lab,sub);var t=el('div','sk-sw'+(checked?' on':''));t.setAttribute('role','switch');
  row.appendChild(t);var st={on:!!checked,row:row,label:L};
  on(row,'click',function(){st.on=!st.on;t.className='sk-sw'+(st.on?' on':'');onChange(st.on);});
  st.set=function(v){st.on=!!v;t.className='sk-sw'+(st.on?' on':'');};
  return st;
};
Group.prototype.choice=function(lab,options,selected,sub,onPick){
  var row=this.row(false,true);label(row,lab,sub);var v=el('div','sk-val',options[selected]!==undefined?options[selected]:'—');row.appendChild(v);
  row.appendChild(icon('chevron','sk-chev down'));
  var cur=selected;
  on(row,'click',function(){blurAll();choose(lab,options,cur,function(i){cur=i;v.textContent=options[i];onPick(i);});});
  return v;
};
Group.prototype.nav=function(lab,value,sub,onClick){
  var row=this.row(false,true);var L=label(row,lab,sub);var v=el('div','sk-val',value);row.appendChild(v);
  row.appendChild(icon('chevron','sk-chev'));on(row,'click',onClick);v.label=L;return v;
};
Group.prototype.info=function(lab,value,sub){var row=this.row();var L=label(row,lab,sub);var v=el('div','sk-val n',value);row.appendChild(v);v.label=L;return v;};
Group.prototype.custom=function(node){var row=this.row();row.appendChild(node);node.style.flex='1';return row;};
Group.prototype.segmented=function(lab,options,selected,onPick){
  var row=this.row();label(row,lab);var bs=[];
  options.forEach(function(o,i){var b=el('button','sk-seg'+(i===selected?' on':''),o);bs.push(b);row.appendChild(b);
    on(b,'click',function(){bs.forEach(function(x,j){x.className='sk-seg'+(j===i?' on':'');});onPick(i);});});
  return bs;
};
function blurAll(){var a=doc.activeElement;if(a&&a.blur&&a!==doc.body)a.blur();}

// ============================================================================
// Shell: the overlay, toolbar, screens stack, bottom nav, dialogs, snackbar
// ============================================================================
var shell=null,stack=[],tab='calc',openFlag=false,skipPop=0,splashShown=false;
function still(){return S().motion==='reduced'||(root.matchMedia&&root.matchMedia('(prefers-reduced-motion: reduce)').matches);}
function isDark(){var t=S().theme;if(t==='dark')return true;if(t==='light')return false;return !!(root.matchMedia&&root.matchMedia('(prefers-color-scheme: dark)').matches);}
function applyTheme(){if(!shell)return;shell.root.classList.toggle('dark',isDark());shell.root.classList.toggle('still',still());}

function build(){
  if(shell)return shell;
  var st=el('style');st.textContent=CSS;doc.head.appendChild(st);
  var r=el('div','hide');r.id='sky';
  var bar=el('div','sk-bar');var main=el('div','sk-main');var res=el('div');var nav=el('div','sk-nav');
  add(r,bar,main,res,nav);doc.body.appendChild(r);
  shell={root:r,bar:bar,main:main,res:res,nav:nav};
  applyTheme();
  if(root.matchMedia){try{root.matchMedia('(prefers-color-scheme: dark)').addEventListener('change',applyTheme);}catch(e){}}
  on(root,'popstate',function(){
    if(!openFlag)return;
    if(skipPop>0){skipPop--;return;}
    dismissOverlays();
    if(stack.length>1){stack.pop();show(-1);}else closeSky(true);
  });
  return shell;
}
function toolbar(o){
  var b=shell.bar;b.innerHTML='';
  if(o.back){var bb=el('button','sk-ibtn');bb.setAttribute('aria-label',o.backLabel||'Back');bb.appendChild(icon(o.backIcon||'back'));on(bb,'click',o.back);b.appendChild(bb);}
  if(o.logo){var lg=el('div','sk-logo');var i1=el('img');i1.src=LOGO.bore;i1.alt='';var i2=el('img','rif');i2.src=LOGO.rifLight;i2.alt='';add(lg,i1,i2);b.appendChild(lg);}
  b.appendChild(el('div','sk-title'+(o.word?' word':''),o.title));
  (o.actions||[]).forEach(function(a){var ab=el('button','sk-ibtn');ab.setAttribute('aria-label',a.label);ab.appendChild(icon(a.icon));on(ab,'click',a.fn);b.appendChild(ab);});
}
function bottomNav(items,current,onPick){
  var n=shell.nav;n.innerHTML='';n.style.display=items?'':'none';if(!items)return;
  items.forEach(function(it){
    var b=el('button','it'+(it.id===current?' on':''));var ind=el('div','ind');ind.appendChild(icon(it.icon));
    add(b,ind,el('div','',it.label));on(b,'click',function(){if(it.id!==current)onPick(it.id);});n.appendChild(b);
  });
}
var TABS=[{id:'calc',label:'Calculator',icon:'target'},{id:'gear',label:'My gear',icon:'gear'},
  {id:'analysis',label:'Analysis',icon:'analysis'},{id:'more',label:'More',icon:'more'}];

// A screen = {render(view), tab?:bool, title...}. push() adds one, back() pops.
function push(screen){stack.push(screen);try{root.history.pushState({sky:stack.length},'');}catch(e){}show(1);}
function back(){if(stack.length>1){try{root.history.back();}catch(e){stack.pop();show(-1);}}else closeSky();}
function replaceTop(screen){stack[stack.length-1]=screen;show(0);}
function current(){return stack[stack.length-1];}
function show(dir){
  dismissOverlays();
  var sc=current();shell.res.innerHTML='';shell.res.className='';
  var scroll=el('div','sk-scroll');var col=el('div','sk-col'+(dir!==0?' sk-in'+(dir<0?' back':''):''));scroll.appendChild(col);
  shell.main.innerHTML='';shell.main.appendChild(scroll);
  sc.scroll=scroll;sc.col=col;
  sc.render(col);
  if(sc.scrollY){var y=sc.scrollY;sc.scrollY=0;scroll.scrollTop=y;}
}
function rerender(keepScroll){var sc=current();if(!sc)return;var y=sc.scroll?sc.scroll.scrollTop:0;sc.col.innerHTML='';shell.res.innerHTML='';sc.render(sc.col);if(keepScroll!==false)sc.scroll.scrollTop=y;}

function dismissOverlays(){if(!shell)return;Array.prototype.slice.call(shell.root.querySelectorAll('.sk-scrim,.sk-menu')).forEach(function(x){x.remove();});}
function scrim(sheet){var s=el('div','sk-scrim'+(sheet?' sheet':''));on(s,'click',function(ev){if(ev.target===s)s.remove();});shell.root.appendChild(s);return s;}
// Material alert dialog
function dialog(o){
  var s=scrim();var d=el('div','sk-dlg');s.appendChild(d);
  if(o.title)d.appendChild(el('div','dt',o.title));
  if(o.message)d.appendChild(el('div','db',o.message));
  if(o.body)d.appendChild(o.body);
  var acts=el('div','da');
  (o.buttons||[{label:'OK'}]).forEach(function(b){acts.appendChild(on(el('button','sk-tbtn',b.label),'click',function(){if(b.keep&&b.fn&&b.fn()===false)return;s.remove();if(b.fn&&!b.keep)b.fn();}));});
  d.appendChild(acts);return {close:function(){s.remove();},el:d};
}
function alertBox(title,msg){dialog({title:title,message:msg});}
function confirmBox(title,msg,yes,fn,no){dialog({title:title,message:msg,buttons:[{label:no||'Cancel'},{label:yes,fn:fn}]});}
function choose(title,options,cur,fn){
  var list=el('div','dl');var dlg;
  options.forEach(function(o,i){var r=el('div','ch');var rb=el('div','sk-radio'+(i===cur?' on':''));add(r,rb,el('div','',o));
    on(r,'click',function(){dlg.close();fn(i);});list.appendChild(r);});
  dlg=dialog({title:title,body:list,buttons:[{label:'Cancel'}]});
}
function ask(title,value,fn,hint){
  var box=el('div','db');var e=el('input','sk-txt');e.type='text';e.value=value||'';e.placeholder=hint||'';box.appendChild(e);
  dialog({title:title,body:box,buttons:[{label:'Cancel'},{label:'OK',fn:function(){var v=e.value.trim();if(v)fn(v);}}]});
  setTimeout(function(){e.focus();e.select();},50);
}
function sheet(title,fill){
  var s=scrim(true);var sh=el('div','sk-sheet');s.appendChild(sh);
  add(sh,el('div','grab'),el('div','st',title));var list=el('div');sh.appendChild(list);
  fill(list,function(){s.remove();});
}
function menu(anchor,items){
  dismissOverlays();
  var m=el('div','sk-menu');var rr=anchor.getBoundingClientRect(),rs=shell.root.getBoundingClientRect();
  items.forEach(function(it){m.appendChild(on(el('div','',it[0]),'click',function(){m.remove();it[1]();}));});
  shell.root.appendChild(m);
  var top=rr.bottom-rs.top-8,left=Math.max(8,rr.right-rs.left-m.offsetWidth);
  if(top+m.offsetHeight>rs.height-8)top=Math.max(8,rr.top-rs.top-m.offsetHeight+8);
  m.style.top=top+'px';m.style.left=left+'px';
  setTimeout(function(){var close=function(ev){if(!m.contains(ev.target)){m.remove();doc.removeEventListener('click',close,true);}};doc.addEventListener('click',close,true);},0);
}
var snackT=null;
function snack(msg,action,fn){
  Array.prototype.slice.call(shell.root.querySelectorAll('.sk-snack')).forEach(function(x){x.remove();});
  var s=el('div','sk-snack');s.appendChild(el('span','',msg));
  if(action)s.appendChild(on(el('button','',action),'click',function(){s.remove();fn();}));
  var navH=shell.nav.style.display==='none'?0:shell.nav.offsetHeight;var resH=shell.res.offsetHeight;
  s.style.bottom=(12+navH+resH)+'px';shell.root.appendChild(s);
  clearTimeout(snackT);snackT=setTimeout(function(){s.remove();},action?5000:2800);
}
function busy(onb){
  var b=shell.main.querySelector('.sk-busy');
  if(!onb){if(b)b.remove();return;}
  if(b)return;b=el('div','sk-busy');var i1=el('img');i1.src=LOGO.bore;var i2=el('img','rif');i2.src=isDark()?LOGO.rifLight:LOGO.rifNavy;add(b,i1,i2);shell.main.appendChild(b);
}
function download(name,mime,text){
  if(root.downloadFile){root.downloadFile(name,mime,text);return;}
  var a=el('a');a.href=URL.createObjectURL(new Blob([text],{type:mime}));a.download=name;doc.body.appendChild(a);a.click();setTimeout(function(){a.remove();},500);
}
function splash(){
  if(splashShown)return;splashShown=true;
  var s=el('div','sk-splash');var lg=el('div','lg');var i1=el('img');i1.src=LOGO.bore;var i2=el('img','rif');i2.src=LOGO.rifNavy;add(lg,i1,i2);
  add(s,lg,el('div','w1','SKYWARD'),el('div','w2','BALLISTICS'),el('div','w3','IN CORIOLIS WIND LAB'));
  shell.root.appendChild(s);
  var gone=false;function hide(){if(gone)return;gone=true;s.style.opacity='0';setTimeout(function(){s.remove();},230);}
  on(s,'click',hide);setTimeout(hide,1000);
}


// ============================================================================
// Wind dial (Skyward WindDial.kt): 12 o'clock = wind from the target.
// ============================================================================
function windDial(angle,onPick,onRelease){
  var NS='http://www.w3.org/2000/svg',S0=176,cx=88,cy=88,r=85,ns=15;
  var svg=doc.createElementNS(NS,'svg');svg.setAttribute('viewBox','0 0 '+S0+' '+S0);svg.setAttribute('class','sk-dial');
  function mk(t,a){var e=doc.createElementNS(NS,t);for(var k in a)e.setAttribute(k,a[k]);svg.appendChild(e);return e;}
  mk('circle',{cx:cx,cy:cy,r:r,fill:'var(--s-lowest)',stroke:'var(--on)','stroke-width':2.5});
  for(var h=0;h<12;h++){var a=h*Math.PI/6,sx=Math.sin(a),sy=-Math.cos(a);
    if(h%3===0){var rr=r-ns*0.95;var t=mk('text',{x:cx+sx*rr,y:cy+sy*rr+ns*0.36,'text-anchor':'middle','font-size':ns,'font-weight':700,fill:'var(--on)'});t.textContent=h===0?'12':''+h;}
    else mk('line',{x1:cx+sx*(r-9),y1:cy+sy*(r-9),x2:cx+sx*(r-2),y2:cy+sy*(r-2),stroke:'var(--outline)','stroke-width':2});}
  mk('circle',{cx:cx,cy:cy,r:5,fill:'var(--on)'});
  var arrow=mk('path',{fill:'var(--cyan)',stroke:'var(--on)','stroke-width':2,'stroke-linejoin':'round'});
  var shown=angle,dragging=false,enabled=true,cur=angle;
  function draw(){var a=shown*Math.PI/180,ux=Math.sin(a),uy=-Math.cos(a),px=-uy,py=ux,tail=r-ns*1.9,tip=11,hl=16,hw=11,sw=4.5;
    function P(k,s){return (cx+ux*k+px*s).toFixed(2)+','+(cy+uy*k+py*s).toFixed(2);}
    arrow.setAttribute('d','M'+P(tip,0)+'L'+P(tip+hl,hw)+'L'+P(tip+hl,sw)+'L'+P(tail,sw)+'L'+P(tail,-sw)+'L'+P(tip+hl,-sw)+'L'+P(tip+hl,-hw)+'Z');}
  draw();
  function at(ev){var b=svg.getBoundingClientRect(),dx=ev.clientX-(b.left+b.width/2),dy=ev.clientY-(b.top+b.height/2);
    if(dx*dx+dy*dy<100)return null;return nrm(Math.atan2(dx,-dy)*180/Math.PI);}
  function move(ev){if(!dragging)return;ev.preventDefault();var d=at(ev);if(d===null)return;shown=d;var sn=nrm(Math.round(d/15)*15);
    if(sn!==cur){cur=sn;onPick(sn);}draw();}
  on(svg,'pointerdown',function(ev){if(!enabled)return;dragging=true;try{svg.setPointerCapture(ev.pointerId);}catch(e){}blurAll();move(ev);});
  on(svg,'pointermove',move);
  function up(){if(!dragging)return;dragging=false;settle(shown,cur);if(onRelease)onRelease();}
  on(svg,'pointerup',up);on(svg,'pointercancel',up);
  function settle(from,to){var turn=((to-from+540)%360)-180;if(!turn||still()){shown=to;draw();return;}
    var t0=null;function step(ts){if(t0===null)t0=ts;var f=Math.min(1,(ts-t0)/140);f=1-(1-f)*(1-f);shown=nrm(from+turn*f);draw();if(f<1)requestAnimationFrame(step);}
    requestAnimationFrame(step);}
  return {el:svg,set:function(v){cur=nrm(v);if(!dragging){shown=cur;draw();}},enable:function(b){enabled=b;svg.setAttribute('class','sk-dial'+(b?'':' off'));}};
}

// ============================================================================
// Phone compass for the direction of fire (DeviceOrientation). Browsers only
// give it to secure pages, so on http://192.168.4.1 it may not be offered.
// ============================================================================
var Compass={
  // true if the browser has the API at all; whether it reports is found by listening
  // On plain http (the base station page) Chrome removes the API entirely, so still offer the
  // button there: the sheet then explains how to allow it.
  maybe:function(){return typeof root.DeviceOrientationEvent!=='undefined'||root.isSecureContext===false;},
  decl:function(){return fin(ls('compassDecl','0'),0);},
  start:function(onHeading,onFail){
    var got=false,evs=['deviceorientationabsolute','deviceorientation'],stopped=false;
    function h(e){
      var hd=null;
      if(typeof e.webkitCompassHeading==='number'&&e.webkitCompassHeading>=0)hd=e.webkitCompassHeading;          // iOS: clockwise from magnetic north
      else if((e.absolute||e.type==='deviceorientationabsolute')&&typeof e.alpha==='number'){                    // Android: alpha counter-clockwise
        var so=(root.screen&&root.screen.orientation&&root.screen.orientation.angle)||root.orientation||0;hd=360-e.alpha+so;}
      if(hd===null)return;got=true;onHeading(nrm(hd+Compass.decl()));
    }
    // Listen straight away: newer Chrome has requestPermission too and may refuse it while
    // still sending events. Only report a failure if no heading arrives.
    var refused=false;
    function wait(){setTimeout(function(){if(!got&&!stopped)onFail(root.isSecureContext===false?'insecure':refused?'denied':'none');},1800);}
    var D=root.DeviceOrientationEvent;
    if(!D){setTimeout(function(){onFail(root.isSecureContext===false?'insecure':'none');},0);return function(){};}
    evs.forEach(function(n){root.addEventListener(n,h,true);});
    if(typeof D.requestPermission==='function'){
      var pr;try{pr=D.requestPermission();}catch(e){pr=null;}
      if(pr&&pr.then)pr.then(function(r){if(r!=='granted')refused=true;wait();},function(){refused=true;wait();});else wait();
    }else wait();
    return function(){stopped=true;evs.forEach(function(n){root.removeEventListener(n,h,true);});};
  }
};
// Bottom sheet: point the phone at the target, read the heading, use it.
function compassSheet(done){
  sheet('Direction of fire from the phone compass',function(list,close){
    var big=el('div','','—°');big.style.cssText='font-size:56px;font-weight:700;text-align:center;color:var(--on);padding:8px 0 0';
    var sub=el('div','sk-note','Hold the phone flat or upright with its top edge pointing at the target. Keep it away from the rifle and other metal.');sub.style.textAlign='center';
    var bw=el('div','sk-bwrap');var use=el('button','sk-btn','Waiting for the compass…');use.disabled=true;bw.appendChild(use);
    var d=Compass.decl(),dl=el('div','sk-note',d?'Includes your declination correction of '+(d>0?'+':'')+Num.trim(d,1)+'°.':'Magnetic heading. If your meters are set to true north, add your declination under More → Settings.');dl.style.textAlign='center';
    add(list,big,sub,bw,dl);
    var samples=[],stop=null,shown=null;
    function avg(){var x=0,y=0;samples.forEach(function(a){x+=Math.cos(a*Math.PI/180);y+=Math.sin(a*Math.PI/180);});return nrm(Math.atan2(y,x)*180/Math.PI);}
    stop=Compass.start(function(hd){
      samples.push(hd);if(samples.length>12)samples.shift();shown=Math.round(avg())%360;
      big.textContent=shown+'°';use.disabled=false;use.textContent='Use '+shown+'°';
    },function(why){
      big.textContent='—';use.disabled=true;use.textContent='Compass not available';
      sub.textContent=why==='insecure'?'This browser only gives the compass to secure (https) pages, and the base station page is http. On Android Chrome you can allow it once: open chrome://flags, find “Insecure origins treated as secure”, add http://192.168.4.1, set it to Enabled and relaunch Chrome. Safari on iPhone has no such setting, so type the direction of fire there.'
        :why==='denied'?'Compass access was refused. Allow motion and orientation access for this page in the browser settings, then try again.'
        :'This device or browser does not report a compass heading. Type the direction of fire instead.';
    });
    on(use,'click',function(){if(shown===null)return;if(stop)stop();close();done(shown);});
    // stop listening when the sheet is dismissed
    var sc=list.parentNode.parentNode;new MutationObserver(function(m,o){if(!sc.isConnected){if(stop)stop();o.disconnect();}}).observe(shell.root,{childList:true});
  });
}
function compassPill(row,onSet){
  if(!Compass.maybe())return;
  var under=el('div','sk-under');var b=el('button','sk-pill','Use phone compass');
  on(b,'click',function(ev){ev.stopPropagation();blurAll();compassSheet(onSet);});under.appendChild(b);row.appendChild(under);
}

function stale(){return !!(root.liveStale&&root.liveStale());}
function liveBadge(){var b=el('span','sk-live');add(b,el('i'),doc.createTextNode('LIVE'));return b;}

// ============================================================================
// CALCULATOR (Skyward CalculatorActivity) + Wind Lab live inputs
// ============================================================================
function CalcScreen(){
  var me={tab:'calc'},live={},gen=0,timer=null,lastSolve=0;
  me.render=function(col){
    var s=S();
    toolbar({back:function(){closeSky();},backLabel:'Back to Wind Lab',logo:true,title:'SKYWARD',word:true,
      actions:[{icon:'settings',label:'Settings',fn:function(){push(SettingsScreen());}}]});
    bottomNav(TABS,'calc',goTab);
    var f=new Form(col),cur=sel();me.cur=cur;live={};
    var p=params();

    f.header('Firearm and ammo');
    f.group(function(g){
      g.nav('Firearm',cur.rifle?cur.rifle.name:'Tap to add a firearm',null,pickRifle);
      var sub=cur.load?[loadFlags(cur.load),bcSummary(cur.load,cur.bullet)].filter(Boolean).join(' · '):null;
      g.nav('Ammo',cur.load?cur.load.name:(cur.rifle?'Tap to add ammo':'Add a firearm first'),sub,pickAmmo);
    });

    f.header('Distance to target');
    f.group(function(g){
      var row=g.stepper('Distance',Q.DISTANCE,fin(getv('range',500),500),function(v){if(v!==null&&v>0&&v<=5000){setv('range',v);changed();}},function(dir){
        var st=S(),stepD=Units.toDisplay(Q.DISTANCE,Math.max(0.1,st.solutionStepYd),st),c=Units.toDisplay(Q.DISTANCE,fin(getv('range',500),500),st);
        var yd=Units.fromDisplay(Q.DISTANCE,(Math.round(c/stepD)+dir)*stepD,st);
        if(yd>0&&yd<=5000){setv('range',yd);row.set(yd);changed();}
      });
    });

    // ---- wind: typed, or live from the Wind Lab meters ----
    f.header('Wind');
    var lw=liveWind(),isLive=liveOn('live_wind');
    f.group(function(g){
      live.windSw=g.sw('Live wind from Wind Lab',isLive,windLiveText(lw,isLive),function(c){
        if(!c){var w=liveWind();if(w.ok){setv('windmph',+w.speed.toFixed(1));setv('windrel',Math.round(w.rel));}}
        setLive('live_wind',c);rerender();changed();});
      live.speed=g.stepper('Wind speed',Q.SPEED,fin(p.windmph,0),function(v){if(v!==null&&v>=0&&v<100){setv('windmph',v);changed();}},function(dir){
        var st=S(),nx=Math.min(99,Math.max(0,Math.round(Units.toDisplay(Q.SPEED,fin(getv('windmph',10),0),st))+dir)),mph=Units.fromDisplay(Q.SPEED,nx,st);
        setv('windmph',mph);live.speed.set(mph);changed();},{badge:isLive?liveBadge():null});
      var box=el('div','sk-dialbox');
      box.appendChild(el('div','top',isLive?'Wind is coming from (live)':'Wind is coming from'));
      live.dial=windDial(fin(p.windrel,90),function(deg){setv('windrel',deg);if(live.angle)live.angle.set(deg);wordsFor(deg);changed(true);},function(){changed();});
      box.appendChild(live.dial.el);
      live.hour=el('div','hr');live.side=el('div','sd');add(box,live.hour,live.side);
      box.appendChild(el('div','tip',isLive?'Follows the meters and your direction of fire':'Tap or drag the dial. 12 = from the target'));
      g.custom(box);
      live.angle=g.number('Exact angle',Q.ANGLE,fin(p.windrel,90),function(v){if(v!==null&&v>=0&&v<=360){setv('windrel',v);live.dial.set(v);wordsFor(v);changed();}},
        {decimals:0,sub:'The dial snaps to the half hour. Type any angle here',width:'84px'});
      live.az=g.stepper('Direction of fire',Q.ANGLE,fireAz(),function(v){if(v!==null&&v>=0&&v<=360){setFireAz(v);changed();}},function(dir){
        setFireAz(fireAz()+dir);live.az.set(fireAz());changed();},{sub:'Compass bearing to the target. Turns the meters’ wind into the wind you shoot through',decimals:0,unit:'°'});
      compassPill(live.az.row,function(v){setFireAz(v);live.az.set(fireAz());changed();snack('Direction of fire set to '+fireAz()+'°');});
      g.number('Second wind speed',Q.SPEED,fin(getv('wind2',0),0),function(v){if(v!==null&&v>=0&&v<100){setv('wind2',v);changed();}},
        {sub:'Optional. For gusts: shows a second windage. 0 = not used'});
    });
    wordsFor(fin(p.windrel,90));
    setLiveEnabled(isLive&&lw.ok);
    if(!isLive)live.az.row.style.display='none';

    f.header('More conditions');
    f.note('Set these once for the day. Tap a card to open it.');
    f.foldable('angle','Slope and moving target',angleSummary,function(g){
      g.number('Uphill / downhill angle',Q.ANGLE,fin(getv('look',0),0),function(v){if(v!==null&&Math.abs(v)<90){setv('look',v);changed();}},
        {signed:true,sub:'Also called look angle. + uphill, − downhill'});
      g.number('Cant',Q.ANGLE,fin(getv('cant',0),0),function(v){if(v!==null&&Math.abs(v)<=90){setv('cant',v);changed();}},{signed:true,sub:'Firearm leaning sideways. + is clockwise'});
      var right=ls('skyMoveRight','1')==='1',ts=fin(getv('tgtMph',0),0);if(ts!==0)right=ts>0;
      g.number('Target speed',Q.SPEED,Math.abs(ts),function(v){if(v!==null&&v>=0){setv('tgtMph',right?v:-v);changed();}},{sub:'0 when the target is not moving'});
      g.segmented('Target moves',['← Left','Right →'],right?1:0,function(i){right=i===1;lsSet('skyMoveRight',right?'1':'0');
        setv('tgtMph',Math.abs(fin(getv('tgtMph',0),0))*(right?1:-1));changed();});
    });
    f.foldable('weather','Weather',weatherSummary,function(g){
      var lt=liveOn('live_temp')&&liveOn('live_pres'),d=wl();
      live.wxSw=g.sw('Live from base station',lt,baseOk()?('Base station: '+Units.text(Q.TEMPERATURE,d.base.tempF,0)+' · '+Units.text(Q.PRESSURE,d.base.presInHg)):'No base station data yet',function(c){
        if(!c&&baseOk()){var b=wl().base;setv('tempF',+b.tempF.toFixed(1));setv('pres',+b.presInHg.toFixed(2));setv('presAbs','1');}
        setLive('live_temp',c);setLive('live_pres',c);rerender();changed();});
      if(S().useDensityAltitude&&!lt){
        g.number('Density altitude',Q.ALTITUDE,DA.of(G().atmoOf(params())),function(v){if(v!==null&&v>-5000&&v<30000){
          var a=DA.atmosphere(v,fin(getv('tempF',59),59));setv('pres',a.pressureInHg);setv('presAbs','1');setv('alt',v);setv('hum',0);changed();}},{signed:true,sub:'One number for how thin the air is'});
        g.number('Temperature',Q.TEMPERATURE,fin(getv('tempF',59),59),function(v){if(v!==null&&v>-80&&v<160){
          var da=DA.of(G().atmoOf(params()));var a=DA.atmosphere(da,v);setv('tempF',v);setv('pres',a.pressureInHg);setv('presAbs','1');changed();}},{signed:true});
      }else{
        live.temp=g.number('Temperature',Q.TEMPERATURE,fin(p.tempF,59),function(v){if(v!==null&&v>-80&&v<160){setv('tempF',v);changed();}},{signed:!lt});
        live.pres=g.number('Pressure',Q.PRESSURE,fin(p.pres,29.92),function(v){if(v!==null&&v>10&&v<35){setv('pres',v);changed();}});
        if(lt){live.temp.setEnabled(false);live.pres.setEnabled(false);}
        else g.sw('Pressure was measured here',getv('presAbs','1')!=='0','On: station (absolute) pressure from a weather meter at your position. Off: sea-level pressure from a forecast',
          function(c){setv('presAbs',c?'1':'0');changed();});
        g.number('Altitude',Q.ALTITUDE,fin(getv('alt',0),0),function(v){if(v!==null&&v>-2000&&v<30000){setv('alt',v);changed();}},{signed:true,sub:lt?'Height above sea level. Used for local gravity':'Height above sea level'});
        g.number('Humidity',Q.HUMIDITY,fin(getv('hum',0),0),function(v){if(v!==null&&v>=0&&v<=100){setv('hum',v);changed();}});
        live.da=g.info('Density altitude','',lt?'From the base station’s temperature and pressure':'Worked out from the values above');
      }
    });
    f.foldable('fine','Fine corrections',fineSummary,function(g){
      g.sw('Spin drift',opt('spin','1')!=='0','Sideways drift caused by the bullet’s spin',function(c){setOpt('spin',c?'1':'0');changed();});
      var models=['LITZ_CLASSIC','MEASURED','LITZ'];
      g.choice('Spin drift method',['Litz (Coriolis default)','Measured (Skyward default)','Classic Litz, zeroed'],Math.max(0,models.indexOf(opt('spinModel','LITZ_CLASSIC'))),null,
        function(i){setOpt('spinModel',models[i]);changed();});
      live.sg=g.info('Bullet stability (Sg)','—','1.4 or more is stable');
      g.sw('Aerodynamic jump',opt('jump','1')!=='0','Small up / down shift caused by a crosswind',function(c){setOpt('jump',c?'1':'0');changed();});
      g.sw('Coriolis',opt('earth','0')==='1','The Earth’s rotation. Matters on very long shots',function(c){setOpt('earth',c?'1':'0');rerender();changed();});
      live.grav=g.sw('Local gravity',opt('localG','0')==='1',gravityText(),function(c){setOpt('localG',c?'1':'0');rerender();changed();});
      if(opt('earth','0')==='1'||opt('localG','0')==='1'){
        var gl=liveOn('live_lat'),hasGps=baseOk()&&wl().base.gps;
        live.lat=g.number('Latitude',Q.ANGLE,fin(p.lat,45),function(v){if(v!==null&&Math.abs(v)<=90){setv('lat',v);changed();}},
          {signed:!gl,sub:gl?(hasGps?'From the base station GPS':'Base station has no GPS fix: using the typed value'):'+ north, − south. Used by Coriolis and local gravity',
           action:[[gl?'Type it instead':'Use base station GPS',function(){setLive('live_lat',!gl);rerender();changed();}]]});
        if(gl&&hasGps)live.lat.setEnabled(false);
      }
      if(opt('earth','0')==='1'){
        var az=g.number('Direction of fire',Q.ANGLE,fireAz(),function(v){if(v!==null&&v>=0&&v<=360){setFireAz(v);changed();}},
          {decimals:0,sub:'Compass bearing (azimuth). 0 = north, 90 = east. Same as the Wind Lab firing azimuth'});
        live.az2=az;
        compassPill(az.row,function(v){setFireAz(v);az.set(fireAz());if(live.az)live.az.set(fireAz());changed();snack('Direction of fire set to '+fireAz()+'°');});
      }
    });

    buildResultBar();
    refreshLive();
  };
  function windLiveText(w,isLive){
    if(!isLive)return 'Off: type the wind below';
    if(!wl())return 'Waiting for the base station…';
    if(!w.ok)return stale()?'Lost the base station: this is not live wind. Rejoin its Wi-Fi or turn live wind off':'No meter has valid wind right now';
    return 'Weighted wind from '+w.count+' meter'+(w.count===1?'':'s')+(w.bad?' ('+w.bad+' with no wind)':'')+', relative to your direction of fire';
  }
  function setLiveEnabled(lv){
    if(live.speed)live.speed.setEnabled(!lv);
    if(live.angle)live.angle.setEnabled(!lv);
    if(live.dial)live.dial.enable(!lv);
  }
  function wordsFor(deg){if(live.hour)live.hour.textContent=WindText.hour(deg);if(live.side)live.side.textContent=WindText.side(deg);}
  function angleSummary(){
    var look=fin(getv('look',0),0),cant=fin(getv('cant',0),0),ts=fin(getv('tgtMph',0),0),parts=[];
    parts.push(look>0?'Uphill '+Num.trim(look,1)+'°':(look<0?'Downhill '+Num.trim(-look,1)+'°':'Level shot'));
    if(cant)parts.push('cant '+Num.trim(cant,1)+'°');
    parts.push(ts===0?'target not moving':'target moving '+(ts>0?'right':'left')+' at '+speedTxt(Math.abs(ts)));
    return parts.join(' · ');
  }
  function weatherSummary(){
    var p=params(),a=G().atmoOf(p),st=S();
    var t=Num.trim(Units.toDisplay(Q.TEMPERATURE,a.temperatureF,st),0)+' '+Units.label(Q.TEMPERATURE,st);
    var lt=liveOn('live_temp')&&liveOn('live_pres')&&baseOk();
    if(st.useDensityAltitude)return (lt?'Live · ':'')+t+' · density altitude '+Units.text(Q.ALTITUDE,DA.of(a),0);
    return (lt?'Live · ':'')+t+' · '+Units.text(Q.PRESSURE,a.pressureInHg)+' · '+Num.trim(a.humidityPct,0)+'% humidity';
  }
  function fineSummary(){
    function oo(b){return b?'on':'off';}
    return 'Spin drift '+oo(opt('spin','1')!=='0')+' · aerodynamic jump '+oo(opt('jump','1')!=='0')+' · Coriolis '+oo(opt('earth','0')==='1')+(opt('localG','0')==='1'?' · local gravity on':'');
  }
  function gravityText(){
    var p=params(),g=SkyEngine.localGravity?SkyEngine.localGravity(fin(p.lat,0),fin(p.alt,0)):9.80665;
    if(opt('localG','0')!=='1')return 'Off: uses standard gravity, 9.807 m/s²';
    return Num.fixed(g,3)+' m/s² at '+Num.fixed(fin(p.lat,0),1)+'°, '+Units.text(Q.ALTITUDE,fin(p.alt,0),0);
  }
  function changed(fast){refreshLive(fast);}
  // Update everything that follows the inputs, then re-solve shortly after.
  function refreshLive(fast){
    var c=me.cur||sel();
    if(live.sg){
      var t='—';
      if(c.complete){try{var p=params(),g=G().engineGear(p),a=G().atmoOf(p);var mv=SkyEngine.muzzleVelocity(g.load,a.temperatureF);var sg=SkyEngine.stability(g.bullet,g.rifle,mv,a);
        if(isFinite(sg)){var n=sg<1?'  not stable':(sg<1.4?'  borderline':'');t=Num.fixed(sg,2)+n;live.sg.className='sk-val n'+(n?' warn':'');}}catch(e){}}
      live.sg.textContent=t;
    }
    if(live.da)live.da.textContent=Units.text(Q.ALTITUDE,DA.of(G().atmoOf(params())),0);
    if(live.grav&&live.grav.label.sub)live.grav.label.sub.textContent=gravityText();
    clearTimeout(timer);timer=setTimeout(solve,fast?60:160);
  }
  me.onData=function(){
    if(!live.speed)return;
    var isLive=liveOn('live_wind'),w=liveWind();
    if(live.windSw&&live.windSw.label.sub)live.windSw.label.sub.textContent=windLiveText(w,isLive);
    if(isLive){setLiveEnabled(w.ok);
      if(w.ok&&doc.activeElement!==live.speed.edit){live.speed.set(w.speed);live.angle.set(w.rel);live.dial.set(w.rel);wordsFor(w.rel);}}
    var lt=liveOn('live_temp')&&liveOn('live_pres');
    if(lt&&baseOk()){var b=wl().base;if(live.temp)live.temp.set(b.tempF);if(live.pres)live.pres.set(b.presInHg);
      if(live.wxSw&&live.wxSw.label.sub)live.wxSw.label.sub.textContent='Base station: '+Units.text(Q.TEMPERATURE,b.tempF,0)+' · '+Units.text(Q.PRESSURE,b.presInHg);}
    if(live.lat&&liveOn('live_lat')&&baseOk()&&wl().base.gps)live.lat.set(wl().base.lat);
    if(live.az&&doc.activeElement!==live.az.edit)live.az.set(fireAz());
    if(Date.now()-lastSolve>500&&(isLive||lt))refreshLive(true);
  };

  // ---- result bar ----
  var cells=null;
  function buildResultBar(){
    var r=shell.res;r.innerHTML='';r.className='';
    var rule=el('div','sk-rule');var bar=el('div','sk-res');var inner=el('div','inner');bar.appendChild(inner);
    var row=el('div','sk-cells');
    function cell(cap,color){var c=el('div','sk-cell');var cp=el('div','c',cap),vl=el('div','vl'),v=el('div','v','—'),u=el('div','u'),sb=el('div','s');
      v.style.color='var('+color+')';add(vl,v,u);add(c,cp,vl,sb);row.appendChild(c);return {box:c,cap:cp,v:v,u:u,s:sb};}
    cells={e:cell('ELEVATION','--elev'),w:cell('WINDAGE','--wind')};
    var hint=el('div','sk-hint');hint.style.display='none';
    var btn=on(el('button','sk-btn','Calculate'),'click',calculate);
    add(inner,row,hint,btn);add(r,rule,bar);cells.row=row;cells.hint=hint;cells.btn=btn;
  }
  function showHint(msg,button){cells.row.style.display='none';cells.hint.style.display='';cells.hint.className='sk-hint';cells.hint.textContent=msg;cells.btn.textContent=button;}
  function solve(){
    if(!cells||current()!==me)return;lastSolve=Date.now();var g0=++gen;
    var c=sel();me.cur=c;
    if(!c.rifle)return showHint('Welcome. Start by adding your firearm.','Add a firearm');
    if(!c.load)return showHint('Next, add the ammo you shoot in '+c.rifle.name+'.','Add ammo');
    if(!c.bullet)return showHint('This ammo’s bullet isn’t available. Import the bullet library or edit the ammo.','Full solution & table');
    var pr=guard(c.rifle,c.load,c.bullet);if(pr)return showHint(pr,'Full solution & table');
    if(liveOn('live_wind')&&wl()&&!liveWind().ok)return showHint(stale()?'Lost the base station, so there is no live wind. Rejoin its Wi-Fi, or turn off live wind and type it.':'No meter has valid wind. Turn off live wind and type it, or check the meters.','Full solution & table');
    var sol;
    try{sol=solveNow();}catch(e){return showHint('Can’t work out this shot: '+(e.message||'check the inputs'),'Full solution & table');}
    if(g0!==gen)return;
    var row=sol.at(fin(sol.p.range,0));if(!row||!(fin(sol.p.range,0)>0))return showHint('Can’t work out this shot: the bullet doesn’t get that far.','Full solution & table');
    var st=S(),u=rifleUnits(c.rifle),pm=st.usePlusMinus,pc=st.outputPrecision;
    var e=Adjust.text(Adjust.dial(row.comeUpMil,u.eu,u.ecorr),'E',pc,u.ec,pm),ec=Adjust.clicksText(Adjust.clicks(row.comeUpMil,u.eu,u.ec,u.ecorr),'E',pm);
    var w=Adjust.text(Adjust.dial(row.windageMil,u.wu,u.wcorr),'W',pc,u.wc,pm),wc=Adjust.clicksText(Adjust.clicks(row.windageMil,u.wu,u.wc,u.wcorr),'W',pm);
    cells.row.style.display='';
    setVal(cells.e.v,e);cells.e.u.textContent=Adjust.unitLabel(u.eu);cells.e.s.textContent=WindText.direction(e)+ec+' clicks';
    cells.w.cap.textContent=fin(sol.p.wind2,0)>0?'WINDAGE (WIND 1)':'WINDAGE';
    setVal(cells.w.v,w);cells.w.u.textContent=Adjust.unitLabel(u.wu);cells.w.s.textContent=WindText.direction(w)+wc+' clicks';
    var ts=fin(sol.p.tgtMph,0),w2=fin(sol.p.wind2,0);
    var extra=(ts!==0&&w2>0)?'Lead and the second wind are on the full solution.':(ts!==0?'The lead for the moving target is on the full solution.':(w2>0?'The second wind is on the full solution.':null));
    cells.hint.style.display=extra?'':'none';cells.hint.className='sk-hint small';cells.hint.textContent=extra||'';
    cells.btn.textContent='Full solution & table';
  }
  function calculate(){
    blurAll();var c=sel();
    if(!c.rifle)return push(RifleEditScreen(null));
    if(!c.load)return push(LoadEditScreen(null,c.rifle.id));
    if(!c.bullet)return snack('This ammo’s bullet isn’t available — edit the ammo and pick a bullet.','Edit',function(){push(LoadEditScreen(c.load.id,c.rifle.id));});
    var pr=guard(c.rifle,c.load,c.bullet);if(pr)return snack(pr);
    push(SolutionScreen('solution'));
  }
  function pickRifle(){quickRifle(function(){rerender();});}
  function pickAmmo(){var c=sel();if(!c.rifle)return pickRifle();quickAmmo(c.rifle.id,function(){rerender();});}
  return me;
}
// Slide a changed number in, like Skyward's Motion.setValue
function setVal(e,t){if(e.textContent===t)return;var old=parseFloat((e.textContent||'').replace(/[^0-9.\-]/g,'')),nw=parseFloat(t.replace(/[^0-9.\-]/g,''));
  e.textContent=t;if(still()||!e.animate)return;var dir=(isFinite(old)&&isFinite(nw)&&nw<old)?-1:1;
  try{e.animate([{transform:'translateY('+(8*dir)+'px)',opacity:.3},{transform:'none',opacity:1}],{duration:150,easing:'ease-out'});}catch(x){}}

// Quick pickers (Skyward QuickPicker: bottom sheets)
function quickRifle(done){
  var rs=rifles();if(!rs.length){push(RifleEditScreen(null));return;}
  var c=sel(),order=recentOrder(rs.map(function(r){return r.id;}),'skyRecentRifles');
  sheet('Firearm',function(list,close){
    order.forEach(function(id){var r=G().rifle(id);if(!r)return;
      list.appendChild(itemRow(r.name,rifleSub(r)+' · '+loadsOf(r.id).length+' ammo',c.rifle&&c.rifle.id===r.id,true,function(){close();useRifle(r.id);done();}));});
    list.appendChild(actRow('All firearms…',function(){close();goTab('gear');}));
    list.appendChild(actRow('Add Firearm',function(){close();push(RifleEditScreen(null));}));
  });
}
function quickAmmo(rid,done){
  var ls2=loadsOf(rid);if(!ls2.length){push(LoadEditScreen(null,rid));return;}
  var c=sel(),r=G().rifle(rid),order=recentOrder(ls2.map(function(l){return l.id;}),'skyRecentLoads');
  sheet('Ammo for '+(r?r.name:''),function(list,close){
    order.forEach(function(id){var l=G().load(id);if(!l)return;
      list.appendChild(itemRow(l.name,loadSub(l,G().bullet(l.bulletId)),c.load&&c.load.id===l.id,true,function(){close();useLoad(l.id);done();}));});
    list.appendChild(actRow('All ammo…',function(){close();goTab('gear');}));
    list.appendChild(actRow('Add Ammo',function(){close();push(LoadEditScreen(null,rid));}));
  });
}
function itemRow(title,sub,selected,big,onClick,menuItems,indent){
  var v=el('div','sk-item');if(indent)v.style.paddingLeft=(16+indent)+'px';
  var ck=icon('check','ck'+(selected?' on':''));v.appendChild(ck);
  var tx=el('div','tx');tx.appendChild(el('div','t'+(big?'':' sm'),title));if(sub)tx.appendChild(el('div','s',sub));v.appendChild(tx);
  if(menuItems){var mo=el('button','mo');mo.setAttribute('aria-label','More for '+title);mo.appendChild(icon('dots'));v.appendChild(mo);
    on(mo,'click',function(ev){ev.stopPropagation();menu(mo,menuItems);});}
  else v.style.paddingRight='16px';
  on(v,'click',onClick);return v;
}
function actRow(text,fn){var a=el('div','sk-act',(/^Add /.test(text)?'+  ':'')+text);a.style.paddingLeft='20px';on(a,'click',fn);return a;}


// ============================================================================
// SOLUTION (Skyward SolutionActivity): Solution / Table / Target Card / Calibrate
// ============================================================================
var SOL_TABS=[{id:'solution',label:'Solution',icon:'target'},{id:'table',label:'Table',icon:'table'},
  {id:'card',label:'Target Card',icon:'card'},{id:'calibrate',label:'Calibrate',icon:'wrench'}];
var SOL_TITLES={solution:'Solution',table:'Table',card:'Target Card',calibrate:'Calibrate'};
function SolutionScreen(which){
  var me={sub:which||'solution',last:{},tableText:'',cardText:''};
  var calib={rows:[{},{},{},{},{}],mode:'mv',result:null};
  me.render=function(col){
    var c=sel();
    if(!c.complete){back();return;}
    var acts=[];
    if(me.sub==='table')acts.push({icon:'share',label:'Share',fn:function(){shareText(me.tableText,'table');}});
    if(me.sub==='card'&&cardsFor(c).length){acts.push({icon:'share',label:'Share',fn:function(){shareText(me.cardText,'card');}});
      acts.push({icon:'dots',label:'More',fn:function(ev){menu(shell.bar.lastChild,[['New card',function(){newCard(c);}],['Rename card',function(){renameCard(c);}],['Delete card',function(){deleteCard(c);}]]);}});}
    toolbar({back:back,title:SOL_TITLES[me.sub],actions:acts});
    bottomNav(SOL_TABS,me.sub,function(id){var order=SOL_TABS.map(function(t){return t.id;});var dir=order.indexOf(id)>order.indexOf(me.sub)?1:-1;me.sub=id;
      col.innerHTML='';col.className='sk-col sk-in'+(dir<0?' back':'');me.render(col);me.scroll.scrollTop=0;});
    if(me.sub==='solution')renderSolution(col,c);
    else if(me.sub==='table')renderTable(col,c);
    else if(me.sub==='card')renderCard(col,c);
    else renderCalibrate(col,c);
  };
  // Live wind: keep the solution following the meters (not while typing)
  me.onData=function(){
    if(me.sub!=='solution'||!liveOn('live_wind'))return;
    if(Date.now()-(me.lastLive||0)<1500)return;
    var a=doc.activeElement;if(a&&(a.tagName==='INPUT'||a.tagName==='TEXTAREA'))return;
    if(shell.root.querySelector('.sk-scrim,.sk-menu'))return;
    me.lastLive=Date.now();rerender();
  };
  function shareText(t,what){
    if(!t)return;
    if(root.navigator&&navigator.share){navigator.share({title:'Skyward Ballistics',text:t}).catch(function(){});return;}
    if(navigator.clipboard&&root.isSecureContext){navigator.clipboard.writeText(t).then(function(){snack('Copied');},function(){download('skyward_'+what+'.txt','text/plain',t);});return;}
    download('skyward_'+what+'.txt','text/plain',t);
  }
  function upd(k,v){setv(k,v);rerender();}

  // ---- Solution ----
  function renderSolution(col,c){
    var sol;try{sol=solveNow();}catch(e){alertBox('Can’t calculate',e.message||String(e));return;}
    var p=sol.p,st=S(),range=fin(p.range,0),row=sol.at(range),u=rifleUnits(c.rifle),pm=st.usePlusMinus,pc=st.outputPrecision;
    if(!row){alertBox('Can’t calculate','The bullet does not reach '+dist(range)+'.');return;}
    var chips=el('div','sk-chips');
    function chip(k,v){var x=el('div','sk-chip');add(x,el('div','k',k),el('div','v',v));chips.appendChild(x);}
    chip('Speed at target',Units.text(Q.VELOCITY,row.velocityFps,0));chip('Energy at target',Units.text(Q.ENERGY,row.energyFtLb,0));chip('Flight time',Num.fixed(row.tofS,2)+' s');
    col.appendChild(chips);
    col.appendChild(el('div','sk-sub',dist(range)+' · '+c.rifle.name+' · '+c.load.name));
    var grid=el('div','sk-grid');col.appendChild(grid);
    function big(cap,mil,axis,unit,click,corr,color){
      var v=Adjust.text(Adjust.dial(mil,unit,corr),axis,pc,click,pm),k=Adjust.clicksText(Adjust.clicks(mil,unit,click,corr),axis,pm);
      var w=el('div');var cd=el('div','sk-big');var sp=el('div','stripe');sp.style.background='var('+color+')';
      var vl=el('div','vl'),vv=el('div','v',v);vv.style.color='var('+color+')';add(vl,vv,el('div','u',Adjust.unitLabel(unit)));
      add(cd,sp,el('div','c',cap),vl,el('div','s',WindText.direction(v)+k+' clicks'));w.appendChild(cd);
      if(me.last[cap]!==undefined&&me.last[cap]!==v&&!still()&&vv.animate){var o=parseFloat(me.last[cap].replace(/[^0-9.\-]/g,'')),n=parseFloat(v.replace(/[^0-9.\-]/g,''));
        try{vv.animate([{transform:'translateY('+(n<o?-8:8)+'px)',opacity:.3},{transform:'none',opacity:1}],{duration:150});}catch(x){}}
      me.last[cap]=v;return w;}
    var w2=fin(p.wind2,0)>0;
    add(grid,big('ELEVATION',row.comeUpMil,'E',u.eu,u.ec,u.ecorr,'--elev'),big(w2?'WINDAGE 1':'WINDAGE',row.windageMil,'W',u.wu,u.wc,u.wcorr,'--wind'));
    var ts=fin(p.tgtMph,0);
    if(ts!==0||w2){
      if(ts!==0)grid.appendChild(big('LEAD (MOVING TARGET)',row.leadMil,'W',u.wu,u.wc,1,'--lead'));
      if(w2&&row.windage2Mil!==null)grid.appendChild(big('WINDAGE 2',row.windage2Mil,'W',u.wu,u.wc,u.wcorr,'--wind2'));
      if(grid.children.length%2)grid.appendChild(el('div'));
    }
    var f=new Form(col);
    f.header('Change the shot');
    f.group(function(g){
      g.custom(sstep('Distance',dist(range),function(dir){var stp=Units.toDisplay(Q.DISTANCE,Math.max(0.1,st.solutionStepYd),st),cur=Units.toDisplay(Q.DISTANCE,range,st);
        var yd=Units.fromDisplay(Q.DISTANCE,(Math.round(cur/stp)+dir)*stp,st);if(yd>0&&yd<=5000)upd('range',yd);}));
      var lw=liveOn('live_wind')&&liveWind().ok;
      g.custom(sstep('Wind speed'+(lw?' (live)':''),speedTxt(fin(p.windmph,0)),lw?null:function(dir){var cur=Units.toDisplay(Q.SPEED,fin(getv('windmph',10),0),st);
        upd('windmph',Units.fromDisplay(Q.SPEED,Math.max(0,Math.round(cur)+dir),st));}));
      g.custom(sstep('Wind angle'+(lw?' (live)':''),Num.trim(fin(p.windrel,90),0)+'° · '+WindText.clock(fin(p.windrel,90)),lw?null:function(dir){
        upd('windrel',nrm((Math.round(fin(getv('windrel',90),90)/30)+dir)*30));}));
    });
    function inches(v){return Num.fixed(Units.toDisplay(Q.LENGTH,v,st),Units.metric(Q.LENGTH,st)?1:2)+' '+Units.label(Q.LENGTH,st);}
    f.header('Details');
    f.group(function(g){
      g.info('Muzzle velocity',Units.text(Q.VELOCITY,sol.muzzleVelocityFps,0),fin(c.load.velocityPerDegFps,0)!==0?'adjusted for powder temperature':null);
      g.info('Drag data used',dragLabel(c.load,c.bullet,p));
      g.info('Mach at target',Num.fixed(row.mach,2),'Bullet speed compared with the speed of sound');
      var sg=sol.stabilityMuzzle,sv=g.info('Bullet stability (Sg)',Num.fixed(sg,2)+(sg<1?' not stable':(sg<1.4?' borderline':'')));if(sg<1.4)sv.className+=' warn';
      g.info('Slows to Mach 1.2 at',sol.transonicYd?dist(sol.transonicYd):'beyond this distance');
      g.info('Goes subsonic at',sol.subsonicYd?dist(sol.subsonicYd):'beyond this distance');
      g.info('Bullet path',inches(row.pathIn));
      g.info('Wind drift',inches(row.windDriftIn));
      if(opt('spin','1')!=='0')g.info('Spin drift',inches(row.spinDriftIn));
      if(opt('jump','1')!=='0')g.info('Aero jump',Num.fixed(milTo(u.eu,row.aeroJumpMil),2)+' '+Adjust.unitLabel(u.eu));
      if(opt('earth','0')==='1'){g.info('Coriolis vertical',inches(row.coriolisVertIn));g.info('Coriolis horizontal',inches(row.coriolisHorizIn));}
      if(fin(p.cant,0)!==0)g.info('Cant',Num.trim(fin(p.cant,0),1)+'°');
      var off=offsets();if(off.e||off.w)g.info('Zero group offset',Num.fixed(Math.abs(off.e*MOA_PER_MIL),2)+' MOA '+(off.e<=0?'high':'low')+', '+Num.fixed(Math.abs(off.w*MOA_PER_MIL),2)+' MOA '+(off.w<=0?'right':'left'),'Added to every hold, like WIND HOLD');
    });
    windHoldBox(f,row,u);
  }
  // Wind Lab extras: aimbot + the meters behind the wind call
  function windHoldBox(f,row,u){
    var d=wl();if(!d)return;
    var lw=liveOn('live_wind'),w=liveWind();
    f.header('Wind Lab');
    f.group(function(g){
      g.info('Wind',lw?(w.ok?speedTxt(w.speed)+' @ '+Math.round(w.rel)+'° · '+w.count+' meter'+(w.count===1?'':'s'):'No valid meter'):'Typed by hand',
        lw?'Weighted average, direction of fire '+fireAz()+'°':'Turn on live wind on the Calculator to follow the meters');
      var moa=row.windageMil*MOA_PER_MIL;
      g.info('WIND HOLD',Math.abs(moa).toFixed(1)+' '+(moa>=0?'R':'L')+' MOA · '+Math.abs(row.windageMil).toFixed(2)+' '+(row.windageMil>=0?'R':'L')+' mil','Same number the Forecast tab and the aimbot use');
      if(d.aim&&d.aim.present&&root.aimGo){
        var b=el('button','sk-btn','AIMBOT '+(moa>=0?'R':'L')+' '+Math.abs(moa).toFixed(1));
        on(b,'click',function(){if(lw&&!w.ok){snack('No valid wind — not dialing');return;}root.aimHoldMOA=moa;root.aimHoldOk=true;root.aimGo();snack('Dialing '+Math.abs(moa).toFixed(1)+' MOA '+(moa>=0?'right':'left'));});
        var wrap=el('div');wrap.style.width='100%';wrap.appendChild(b);g.custom(wrap);
      }
    });
  }
  function sstep(lab,val,fn){
    var c=el('div','sk-sstep');c.appendChild(el('div','lab',lab));var r=el('div','row');
    var m=stepBtn('−','Decrease '+lab,function(){fn(-1);}),pl=stepBtn('+','Increase '+lab,function(){fn(1);});
    if(!fn){m.disabled=true;pl.disabled=true;}
    add(r,m,el('div','val',val),pl);c.appendChild(r);return c;
  }
  function dragLabel(l,b,p){
    var model=l.dragModelOverride||b.dragModel||'G7',base;
    if(model==='CUSTOM')base='Measured drag curve';
    else if(l.bcSeason){base='G7 '+(l.bcSeason==='auto'?'seasonal (auto)':l.bcSeason+' rows');}
    else{var rows=(l.bcPointsOverride&&l.bcPointsOverride.length)?l.bcPointsOverride:(b.bcPoints||[]);
      base=rows.length===1?model+' '+Num.trim(rows[0].bc,3):model+', '+rows.length+' BCs';}
    var ds=fin(l.dragScaleFactor,1);return ds!==1?base+' × '+Num.trim(ds,3):base;
  }

  // ---- Table ----
  function renderTable(col,c){
    var st=S(),maxR=Math.min(5000,Math.max(50,st.tableMaxYd)),step=Math.max(Math.min(1000,Math.max(1,st.tableStepYd)),maxR/400);
    var range=fin(getv('range',500),500),zero=fin(c.load.zeroRangeYd,100);
    var sol;try{sol=solveNow(null,{maxRangeYd:maxR,stepYd:step,extraRangesYd:[range,zero]});}catch(e){col.appendChild(el('div','sk-empty','Can’t calculate: '+(e.message||e)));return;}
    var p=sol.p,u=rifleUnits(c.rifle),pm=st.usePlusMinus,pc=st.outputPrecision,lenU=Units.label(Q.LENGTH,st),mL=Units.metric(Q.LENGTH,st);
    var txt=[];
    function block(title,pairs){
      col.appendChild(el('div','sk-blk',title));txt.push('== '+title+' ==');
      var kv=el('div','sk-kv');pairs.forEach(function(x){add(kv,el('b','',x[0]),el('span','',x[1]));});col.appendChild(kv);
      txt.push(pairs.map(function(x){return x[0]+': '+x[1];}).join('   '));
    }
    var r=c.rifle,a=G().atmoOf(p);
    block('Firearm: '+r.name,[['Twist','1:'+Num.trim(fin(r.twistIn,0),2)+'in '+(r.twistDirection==='LEFT'?'L':'R')],['Sight height',Num.trim(Units.toDisplay(Q.LENGTH,fin(r.sightHeightIn,0),st),2)+' '+lenU],
      ['Sight offset',Num.trim(Units.toDisplay(Q.LENGTH,fin(r.sightOffsetIn,0),st),2)+' '+lenU],['CF (elev/wind)',Num.trim(u.ecorr,3)+'/'+Num.trim(u.wcorr,3)]]);
    block('Ammo: '+c.load.name,[['Bullet',Num.trim(fin(c.bullet.weightGr,0),1)+'gr '+Num.trim(fin(c.bullet.diameterIn,0),3)+'"'],['Drag',dragLabel(c.load,c.bullet,p)],
      ['Muzzle vel',Units.text(Q.VELOCITY,sol.muzzleVelocityFps,0)],['Zero range',dist(zero)]]);
    block('Atmosphere',[['Altitude',Units.text(Q.ALTITUDE,a.altitudeFt,0)],['Temperature',Num.trim(Units.toDisplay(Q.TEMPERATURE,a.temperatureF,st),1)+Units.label(Q.TEMPERATURE,st)],
      ['Pressure',Units.text(Q.PRESSURE,a.pressureInHg)+(a.pressureIsAbsolute?' abs':'')],['Humidity',Num.trim(a.humidityPct,0)+'%']]);
    var ts=fin(p.tgtMph,0);
    block('Shot data',[['Distance',dist(range)],['Look angle',Num.trim(fin(p.look,0),1)+'°'],
      ['Wind',speedTxt(fin(p.windmph,0))+' @ '+Num.trim(fin(p.windrel,90),0)+'°'+(liveOn('live_wind')?' (live)':'')],
      ['Target',ts===0?'Stationary':speedTxt(Math.abs(ts))+(ts>0?' R':' L')],['Spin drift',opt('spin','1')!=='0'?'On':'Off'],
      ['Coriolis',opt('earth','0')==='1'?'On (lat '+Num.trim(fin(p.lat,0),1)+'°, az '+fireAz()+'°)':'Off']]);
    var eu=Adjust.unitLabel(u.eu),wu=Adjust.unitLabel(u.wu);
    var heads=['Range\n('+Units.label(Q.DISTANCE,st)+')','Path\n('+lenU+')','Path\n('+eu+')','Drift\n('+lenU+')','Drift\n('+wu+')','Vel\n('+Units.label(Q.VELOCITY,st)+')','Energy\n('+Units.label(Q.ENERGY,st)+')','TOF\n(s)'];
    var wrap=el('div','sk-tscroll');var t=el('table','sk-tbl');var tr=el('tr');heads.forEach(function(h){tr.appendChild(el('th','',h));});t.appendChild(tr);
    txt.push('');txt.push(heads.map(function(h){return h.replace('\n',' ');}).join('\t'));
    var rows=sol.rows.filter(function(x){return x.rangeYd>0;});if(st.tableSortDescending)rows=rows.reverse();
    rows.forEach(function(x,i){
      var isT=Math.abs(x.rangeYd-range)<1e-6,isZ=Math.abs(x.rangeYd-zero)<1e-6;
      var vals=[(isZ?'◎ ':'')+Num.trim(Units.toDisplay(Q.DISTANCE,x.rangeYd,st),0),Num.fixed(Units.toDisplay(Q.LENGTH,x.pathIn,st),mL?1:2),
        Adjust.text(Adjust.dial(x.comeUpMil,u.eu,u.ecorr),'E',pc,u.ec,pm),Num.fixed(Units.toDisplay(Q.LENGTH,x.windageIn,st),mL?1:2),
        Adjust.text(Adjust.dial(x.windageMil,u.wu,u.wcorr),'W',pc,u.wc,pm),Num.trim(Units.toDisplay(Q.VELOCITY,x.velocityFps,st),0),
        Num.trim(Units.toDisplay(Q.ENERGY,x.energyFtLb,st),0),Num.fixed(x.tofS,2)];
      var r2=el('tr',isT?'tg':(i%2?'st':''));vals.forEach(function(v){r2.appendChild(el('td','',v));});t.appendChild(r2);txt.push(vals.join('\t'));
    });
    wrap.appendChild(t);col.appendChild(wrap);
    var off=offsets();
    col.appendChild(el('div','sk-note','Holds include the zero group offset'+((off.e||off.w)?'':' (none set)')+'. ◎ marks the zero distance; the highlighted row is your target. Row spacing and the furthest distance are in Settings.'));
    me.tableText='Skyward Ballistics — '+r.name+' / '+c.load.name+'\n'+txt.join('\n');
  }

  // ---- Target card (cards live in the gear file, so they sync with Skyward) ----
  function cardsFor(c){return (G().lib().cards||[]).filter(function(x){return x.rifleId===c.rifle.id&&x.loadId===c.load.id;})
    .sort(function(a,b){return (a.name||'').toLowerCase()<(b.name||'').toLowerCase()?-1:1;});}
  function curCard(c){var cs=cardsFor(c);return cs.filter(function(x){return x.id===me.cardId;})[0]||cs[0]||null;}
  function newCard(c){ask('New target card',c.load.name+' card',function(n){me.cardId=G().saveRecord('card',{name:n,rifleId:c.rifle.id,loadId:c.load.id,targets:[]});rerender();});}
  function renameCard(c){var k=curCard(c);if(!k)return;ask('Rename card',k.name,function(n){k=JSON.parse(JSON.stringify(k));k.name=n;G().saveRecord('card',k);rerender();});}
  function deleteCard(c){var k=curCard(c);if(!k)return;confirmBox('Delete card','Delete "'+k.name+'"?','Delete',function(){G().removeRecord('card',k.id);me.cardId=null;rerender();});}
  function renderCard(col,c){
    var f=new Form(col),st=S(),cs=cardsFor(c),k=curCard(c);
    if(!k){f.header('Target Card');
      f.note('You have no target card for '+c.rifle.name+' / '+c.load.name+' yet.\n\nA target card is a list of your targets (distance, slope, and optionally their own wind or movement). It shows the elevation and windage for every target at once, using today’s weather.');
      f.button('Create target card',function(){newCard(c);});return;}
    me.cardId=k.id;
    var p=params(),a=G().atmoOf(p);
    var atmo=Num.trim(Units.toDisplay(Q.TEMPERATURE,a.temperatureF,st),0)+Units.label(Q.TEMPERATURE,st)+' · '+Units.text(Q.PRESSURE,a.pressureInHg)+' · DA '+Units.text(Q.ALTITUDE,DA.of(a),0);
    f.header('Card');
    f.group(function(g){
      if(cs.length>1)g.choice('Card',cs.map(function(x){return x.name;}),cs.indexOf(k),null,function(i){me.cardId=cs[i].id;rerender();});else g.info('Card',k.name);
      g.info('Weather used',atmo);
    });
    var u=rifleUnits(c.rifle),pm=st.usePlusMinus,pc=st.outputPrecision;
    f.header('Targets ('+Adjust.unitLabel(u.eu)+' / '+Adjust.unitLabel(u.wu)+')');
    var txt=[k.name,c.rifle.name+' / '+c.load.name,atmo,'Elevation '+Adjust.unitLabel(u.eu)+', windage '+Adjust.unitLabel(u.wu),''];
    if(!(k.targets||[]).length)f.note('This card has no targets yet. Tap "Add target" below. Tap a target later to change or delete it.');
    else f.group(function(g){
      k.targets.forEach(function(t,i){
        var rw=el('div','sk-trow');var left=el('div');left.style.flex='1';add(left,el('div','n',t.name||dist(t.distanceYd)),el('div','d',describe(t)));
        var right=el('div','r');var ev=el('div'),wv=el('div');ev.style.color='var(--elev)';wv.style.color='var(--wind)';add(right,ev,wv);add(rw,left,right);
        try{
          var over={range:t.distanceYd,look:fin(t.lookAngleDeg,0),wind2:0,tgtMph:fin(t.targetSpeedMph,0)};
          if(t.windSpeedMph!==null&&t.windSpeedMph!==undefined)over.windmph=t.windSpeedMph;
          if(t.windAngleDeg!==null&&t.windAngleDeg!==undefined)over.windrel=t.windAngleDeg;
          var sl=solveNow(over),x=sl.at(t.distanceYd);
          var e=Adjust.text(Adjust.dial(x.comeUpMil,u.eu,u.ecorr),'E',pc,u.ec,pm),w=Adjust.text(Adjust.dial(x.windageMil,u.wu,u.wcorr),'W',pc,u.wc,pm);
          ev.textContent=e+'  ('+Adjust.clicksText(Adjust.clicks(x.comeUpMil,u.eu,u.ec,u.ecorr),'E',pm)+')';
          var wl2=w+'  ('+Adjust.clicksText(Adjust.clicks(x.windageMil,u.wu,u.wc,u.wcorr),'W',pm)+')';
          if(fin(t.targetSpeedMph,0)!==0)wl2+='\nlead '+Adjust.text(milTo(u.wu,x.leadMil),'W',pc,u.wc,pm);
          wv.textContent=wl2;
          txt.push((t.name||dist(t.distanceYd))+' ('+describe(t)+'): elev '+e+' / wind '+w);
        }catch(err){ev.textContent='—';wv.textContent='—';}
        on(rw,'click',function(){editTarget(c,k,i);});
        g.custom(rw);
      });
    });
    f.button('Add target',function(){editTarget(c,k,-1);});
    f.note('Wind left blank on a target uses the main screen’s wind ('+speedTxt(fin(p.windmph,0))+' @ '+Num.trim(fin(p.windrel,90),0)+'°'+(liveOn('live_wind')?', live':'')+').');
    me.cardText=txt.join('\n');
  }
  function describe(t){
    var parts=[dist(t.distanceYd)];
    if(fin(t.lookAngleDeg,0))parts.push(Num.trim(t.lookAngleDeg,1)+'°');
    var hw=t.windSpeedMph!==null&&t.windSpeedMph!==undefined,ha=t.windAngleDeg!==null&&t.windAngleDeg!==undefined;
    if(hw||ha)parts.push('wind '+(hw?speedTxt(t.windSpeedMph):'current')+' @ '+(ha?Num.trim(t.windAngleDeg,0)+'°':'current'));
    if(fin(t.targetSpeedMph,0))parts.push('moving '+speedTxt(Math.abs(t.targetSpeedMph))+(t.targetSpeedMph>0?' R':' L'));
    return parts.join(' · ');
  }
  function editTarget(c,k,i){
    var t=i>=0?JSON.parse(JSON.stringify(k.targets[i])):{name:'',distanceYd:fin(getv('range',500),500),lookAngleDeg:0,windSpeedMph:null,windAngleDeg:null,targetSpeedMph:0};
    var box=el('div','db');var g=new Group(box);
    g.text('Name',t.name,function(v){t.name=v;},{hint:'optional, e.g. Steel 3'});
    g.number('Distance',Q.DISTANCE,t.distanceYd,function(v){t.distanceYd=v;});
    g.number('Slope',Q.ANGLE,t.lookAngleDeg,function(v){t.lookAngleDeg=v||0;},{signed:true,sub:'+ uphill, − downhill'});
    g.number('Wind speed',Q.SPEED,t.windSpeedMph,function(v){t.windSpeedMph=v;},{hint:'current'});
    g.number('Wind angle',Q.ANGLE,t.windAngleDeg,function(v){t.windAngleDeg=v;},{decimals:0,hint:'current'});
    g.number('Target speed',Q.SPEED,t.targetSpeedMph,function(v){t.targetSpeedMph=v||0;},{signed:true,sub:'+ right, − left. 0 = not moving'});
    var btns=[{label:'Cancel'}];
    if(i>=0)btns.unshift({label:'Delete',fn:function(){var kk=JSON.parse(JSON.stringify(k));kk.targets.splice(i,1);G().saveRecord('card',kk);rerender();}});
    btns.push({label:'Save',keep:true,fn:function(){
      if(!(t.distanceYd>0&&t.distanceYd<=5000)){snack('Enter a distance');return false;}
      var kk=JSON.parse(JSON.stringify(k));kk.targets=kk.targets||[];if(i>=0)kk.targets[i]=t;else kk.targets.push(t);
      G().saveRecord('card',kk);rerender();return true;}});
    dialog({title:i>=0?'Target':'Add target',body:box,buttons:btns});
  }

  // ---- Calibrate (Skyward Truing): fit muzzle velocity or drag scale to real come-ups ----
  function renderCalibrate(col,c){
    var f=new Form(col),st=S(),u=rifleUnits(c.rifle),p=params(),a=G().atmoOf(p);
    f.header('Current conditions');
    f.group(function(g){
      g.info('Weather',Units.text(Q.TEMPERATURE,a.temperatureF,0)+' · '+Units.text(Q.PRESSURE,a.pressureInHg));
      g.info('Muzzle velocity',Units.text(Q.VELOCITY,SkyEngine.muzzleVelocity(G().engineGear(p).load,a.temperatureF),0));
      g.info('Drag scale factor',Num.trim(fin(c.load.dragScaleFactor,1),4));
    });
    f.header('Match the app to your real hits');
    f.group(function(g){g.choice('Adjust',['Muzzle velocity (most shooters)','Drag scale factor (BC)'],calib.mode==='mv'?0:1,
      'Velocity for mid distances, drag for the far ones',function(i){calib.mode=i===0?'mv':'drag';calib.result=null;rerender();});});
    f.note('Shoot at known distances, then type the distance and the elevation you really had to dial to hit. The app finds the '+(calib.mode==='mv'?'muzzle velocity':'drag scale factor')+' that matches best.');
    f.header('Your hits');
    var preds=[];
    f.group(function(g){
      var hd=el('div');hd.style.cssText='display:flex;width:100%;font-weight:700;font-size:15px;color:var(--on-var)';
      add(hd,el('div','','Distance ('+Units.label(Q.DISTANCE,st)+')'),el('div','','Dialed ('+Adjust.unitLabel(u.eu)+')'),el('div','','App says'));
      Array.prototype.forEach.call(hd.children,function(x){x.style.flex='1';x.style.textAlign='center';});g.custom(hd);
      calib.rows.forEach(function(r,i){
        var line=el('div');line.style.cssText='display:flex;width:100%;align-items:center;gap:8px';
        var d=numInput(false,'');d.style.cssText='flex:1;width:auto;min-width:0;margin:0';d.value=r.d!==undefined&&r.d!==null?Num.trim(Units.toDisplay(Q.DISTANCE,r.d,st),0):'';
        var m=numInput(true,'');m.style.cssText='flex:1;width:auto;min-width:0;margin:0';m.value=r.m!==undefined&&r.m!==null?Num.trim(r.m,2):'';
        var pr=el('div','','—');pr.style.cssText='flex:1;text-align:center;font-size:18px';preds.push(pr);
        on(d,'input',function(){var v=Num.parse(d.value);r.d=v===null?null:Units.fromDisplay(Q.DISTANCE,v,st);predict();});
        on(m,'input',function(){r.m=Num.parse(m.value);});
        add(line,d,m,pr);g.custom(line);
      });
    });
    function predict(){
      calib.rows.forEach(function(r,i){
        if(!(r.d>0&&r.d<=5000)){preds[i].textContent='—';return;}
        try{var x=solveNow({range:r.d}).at(r.d);preds[i].textContent=Adjust.text(Adjust.dial(x.comeUpMil,u.eu,u.ecorr),'E','hundredth',u.ec,st.usePlusMinus);}catch(e){preds[i].textContent='—';}
      });
    }
    predict();
    f.button('Calibrate',function(){
      blurAll();
      var imp=calib.rows.filter(function(r){return r.d>0&&r.d<=5000&&r.m!==null&&r.m!==undefined&&isFinite(r.m);})
        .map(function(r){return {rangeYd:r.d,actualComeUpMil:r.m/(milTo(u.eu,1)*(u.ecorr>0?u.ecorr:1))-offsets().e};});
      if(!imp.length){alertBox('Calibrate','Enter at least one distance with the elevation that hit.');return;}
      try{var pp=params(),g=G().engineGear(pp),shot=shotOf(pp);
        calib.result=calib.mode==='mv'?SkyEngine.trueMuzzleVelocity(g.rifle,g.load,g.bullet,shot,imp):SkyEngine.trueDragScale(g.rifle,g.load,g.bullet,shot,imp);
        calib.result.imp=imp;calib.result.mode=calib.mode;rerender();
      }catch(e){alertBox('Calibrate',e.message||String(e));}
    });
    var res=calib.result;
    if(res){
      var toDial=function(mil){return Adjust.dial(mil,u.eu,u.ecorr);};
      f.header('Result');
      f.group(function(g){
        if(res.mode==='mv')g.info('New Muzzle Velocity',Units.text(Q.VELOCITY,res.value,0),'at '+Units.text(Q.TEMPERATURE,a.temperatureF,0)+' (was '+Units.text(Q.VELOCITY,SkyEngine.muzzleVelocity(G().engineGear(p).load,a.temperatureF),0)+')');
        else g.info('New Drag Scale Factor',Num.fixed(res.value,4),'was '+Num.trim(fin(c.load.dragScaleFactor,1),4));
        g.info('Typical remaining error',Num.fixed(toDial(res.rmsBeforeMil),2)+' → '+Num.fixed(toDial(res.rmsAfterMil),2)+' '+Adjust.unitLabel(u.eu));
        res.imp.forEach(function(im,i){var off=offsets().e;g.info(dist(im.rangeYd),'actual '+Num.fixed(toDial(im.actualComeUpMil+off),2)+' · before '+Num.fixed(toDial(res.predictedBefore[i]+off),2)+' · after '+Num.fixed(toDial(res.predictedAfter[i]+off),2));});
      });
      f.button('Apply to ammo',function(){
        var l=JSON.parse(JSON.stringify(c.load));
        var what=res.mode==='mv'?'Set the muzzle velocity of "'+l.name+'" to '+Units.text(Q.VELOCITY,res.value,0)+' at this temperature?':'Set the drag scale factor of "'+l.name+'" to '+Num.fixed(res.value,4)+'?';
        confirmBox('Apply to ammo',what,'Apply',function(){
          if(res.mode==='mv')l.muzzleVelocityFps=Math.round(res.loadMuzzleVelocityFps*10)/10;else l.dragScaleFactor=Math.round(res.value*10000)/10000;
          G().saveRecord('load',l);refreshActive();calib.result=null;snack('Saved to "'+l.name+'"');rerender();
        });
      });
    }
  }
  return me;
}


// ============================================================================
// Tabs
// ============================================================================
function screenFor(id){return id==='gear'?GearScreen():id==='analysis'?AnalysisScreen():id==='more'?MoreScreen():CalcScreen();}
function goTab(id){
  tab=id;lsSet('skyTab',id);
  var n=stack.length-1;
  if(n>0){stack=[stack[0]];skipPop++;try{root.history.go(-n);}catch(e){skipPop--;}}
  replaceTop(screenFor(id));
}
function toWindLab(){closeSky();}

// ============================================================================
// Drag data (Skyward CalcLogic.kt BcSelection / BcEditor.kt BcRows)
// ============================================================================
var DS={CURVE:'Measured drag curve',BULLET:'Bullet’s BC rows',COLD:'Cold-weather BC rows (< 30°F)',MILD:'Mild-weather BC rows',
  HOT:'Hot-weather BC rows (> 90°F)',AUTO:'Seasonal rows by temperature (auto)',CUSTOM:'My own BC rows'};
var DS_SEASON={COLD:'cold',MILD:'mild',HOT:'hot'};
function seasons(b){var s=(b&&b.seasonalBcPoints)||{},o=[];['cold','mild','hot'].forEach(function(k){if(s[k]&&s[k].length)o.push(k);});return o;}
function hasCurve(b){return !!(b&&b.dragCurve&&b.dragCurve.length>=2);}
function bcChoices(b){var o=[];if(hasCurve(b))o.push('CURVE');if(b&&b.bcPoints&&b.bcPoints.length)o.push('BULLET');
  var s=seasons(b);if(s.indexOf('cold')>=0)o.push('COLD');if(s.indexOf('mild')>=0)o.push('MILD');if(s.indexOf('hot')>=0)o.push('HOT');if(s.length)o.push('AUTO');o.push('CUSTOM');return o;}
function bcDefault(b){return hasCurve(b)?'CURVE':(b&&b.bcPoints&&b.bcPoints.length)?'BULLET':'CUSTOM';}
function sameRows(a,b){if(!a||!b||a.length!==b.length)return false;
  var sa=a.slice().sort(function(x,y){return y.minVelFps-x.minVelFps;}),sb=b.slice().sort(function(x,y){return y.minVelFps-x.minVelFps;});
  for(var i=0;i<sa.length;i++)if(Math.abs(sa[i].bc-sb[i].bc)>1e-9||Math.abs(sa[i].minVelFps-sb[i].minVelFps)>1e-6)return false;return true;}
function bcDetect(l,b){
  if(l.dragModelOverride==='CUSTOM')return 'CURVE';
  if(l.bcSeason==='auto')return 'AUTO';if(l.bcSeason==='cold')return 'COLD';if(l.bcSeason==='mild')return 'MILD';if(l.bcSeason==='hot')return 'HOT';
  var rows=l.bcPointsOverride;
  if(!rows||!rows.length){if(!l.dragModelOverride||l.dragModelOverride===b.dragModel)return b.dragModel==='CUSTOM'?'CURVE':'BULLET';return 'CUSTOM';}
  if(!l.dragModelOverride||l.dragModelOverride==='G7'){
    var s=b.seasonalBcPoints||{};
    if(sameRows(rows,s.cold))return 'COLD';if(sameRows(rows,s.mild))return 'MILD';if(sameRows(rows,s.hot))return 'HOT';
    if(b.dragModel==='G7'&&sameRows(rows,b.bcPoints))return 'BULLET';
  }
  return 'CUSTOM';
}
function bcApply(l,b,src,model,rows){
  l=JSON.parse(JSON.stringify(l));
  if(src==='CURVE'){l.dragModelOverride=b.dragModel==='CUSTOM'?null:'CUSTOM';l.bcPointsOverride=null;l.bcSeason=null;}
  else if(src==='BULLET'){l.dragModelOverride=null;l.bcPointsOverride=null;l.bcSeason=null;}
  else if(src==='CUSTOM'){l.dragModelOverride=model;l.bcPointsOverride=rows;l.bcSeason=null;}
  else{l.dragModelOverride=b.dragModel==='G7'?null:'G7';l.bcPointsOverride=null;l.bcSeason=DS_SEASON[src]||'auto';}
  return l;
}
function bcValidate(rows){
  if(!rows.length)return 'Enter at least one ballistic coefficient.';
  if(rows.some(function(r){return !(r.bc>0)||r.bc>2;}))return 'Each BC must be between 0 and 2.';
  if(rows.some(function(r){return r.minVelFps<0;}))return 'Min velocities can’t be negative.';
  if(!rows.some(function(r){return r.minVelFps===0;}))return 'One BC must have a min velocity of 0.';
  var seen={};for(var i=0;i<rows.length;i++){if(seen[rows[i].minVelFps])return 'Two BC rows have the same min velocity.';seen[rows[i].minVelFps]=1;}
  return null;
}
function bcDrafts(rows){var d=(rows||[]).slice().sort(function(a,b){return b.minVelFps-a.minVelFps;}).map(function(r){return {bc:r.bc,v:r.minVelFps};});return d.length?d:[{bc:null,v:0}];}
function bcFinish(d){
  var f=d.filter(function(r){return r.bc!==null||r.v!==null;});
  if(f.some(function(r){return r.bc===null;}))return {err:'A BC row has a min velocity but no BC.'};
  var rows;
  if(f.length===1)rows=[{bc:f[0].bc,minVelFps:f[0].v===null?0:f[0].v}];
  else{if(f.some(function(r){return r.v===null;}))return {err:'With several BCs, every row needs a min velocity (one of them 0).'};
    rows=f.map(function(r){return {bc:r.bc,minVelFps:r.v};});}
  var e=bcValidate(rows);if(e)return {err:e};
  return {rows:rows.sort(function(a,b){return b.minVelFps-a.minVelFps;})};
}
function bcRowsText(rows){var s=S();return (rows||[]).slice().sort(function(a,b){return b.minVelFps-a.minVelFps;}).map(function(r){
  return Num.fixed(r.bc,3)+'   ≥ '+Num.trim(Units.toDisplay(Q.VELOCITY,r.minVelFps,s),0)+' '+Units.label(Q.VELOCITY,s);}).join('\n')||'—';}
// The BC rows editor
function bcRowsView(d,onStructure){
  var s=S(),velU=Units.label(Q.VELOCITY,s),box=el('div');box.style.width='100%';
  var cap=el('div','sk-bcrow');var c1=el('div','',('BC')),c2=el('div','','Min Vel ('+velU+')');c1.style.flex='1';c2.style.flex='1.3';
  [c1,c2].forEach(function(c){c.style.fontSize='15px';c.style.color='var(--on-var)';});var sp=el('div');sp.style.width='44px';add(cap,c1,c2,sp);box.appendChild(cap);
  d.forEach(function(r,i){
    var line=el('div','sk-bcrow');line.style.marginTop='6px';
    var a=numInput(false,'BC');a.value=r.bc===null?'':Num.trim(r.bc,4);a.style.flex='1';
    var b=numInput(false,'Min Vel ('+velU+')');b.value=r.v===null?'':Num.trim(Units.toDisplay(Q.VELOCITY,r.v,s),0);b.style.flex='1.3';
    on(a,'input',function(){r.bc=Num.parse(a.value);});
    on(b,'input',function(){var v=Num.parse(b.value);r.v=v===null?null:Units.fromDisplay(Q.VELOCITY,v,s);});
    var x=el('button','sk-x');x.setAttribute('aria-label','Remove row');x.appendChild(icon('close'));x.disabled=d.length<2;if(x.disabled)x.style.opacity='.3';
    on(x,'click',function(){if(d.length>1){d.splice(i,1);onStructure();}});
    add(line,a,b,x);box.appendChild(line);
  });
  var addB=el('button','sk-tbtn');add(addB,icon('add'),doc.createTextNode(' Add BC row'));addB.style.display='flex';addB.style.alignItems='center';addB.style.marginTop='6px';addB.style.paddingLeft='0';
  on(addB,'click',function(){d.push({bc:null,v:null});onStructure();});box.appendChild(addB);
  return box;
}
function bulletTitle2(b){return (b.maker&&(b.name||'').toLowerCase().indexOf(b.maker.toLowerCase())!==0)?(b.maker+' '+(b.name||'')):(b.name||'Bullet');}
function bulletSub(b){
  var p=[Num.trim(fin(b.diameterIn,0),3)+' in',Num.trim(fin(b.weightGr,0),1)+' gr'];
  if(b.lengthIn>0)p.push('L '+Num.trim(b.lengthIn,3));
  if(hasCurve(b))p.push('drag curve');
  if(b.bcPoints&&b.bcPoints.length){var m=b.dragModel==='CUSTOM'?'G7':(b.dragModel||'G7');p.push(b.bcPoints.length===1?m+' '+Num.trim(b.bcPoints[0].bc,3):m+' ('+b.bcPoints.length+' BCs)');}
  if(seasons(b).length)p.push('seasonal');
  if(b.collection)p.push(b.collection);
  return p.join(' · ');
}
function bulletSource(b){return b.source!=='library'?'My bullet':(!b.collection?'Bullet library':(b.collection.length<=24?b.collection:b.collection.slice(0,23)+'…'));}
function bulletMatches(b,q){
  var t=(q||'').toLowerCase().split(/\s+/).filter(Boolean);if(!t.length)return true;
  var h=[b.maker,b.name,b.caliber,b.category,b.collection,Num.trim(fin(b.weightGr,0),1)+'gr',Num.trim(fin(b.weightGr,0),1)+' gr',Num.trim(fin(b.diameterIn,0),3),b.dragModel].join(' ').toLowerCase();
  return t.every(function(x){return h.indexOf(x)>=0;});
}
function bulletsOrdered(){
  return G().bullets().slice().sort(function(a,b){
    var la=a.source==='library'?1:0,lb=b.source==='library'?1:0;if(la!==lb)return la-lb;
    if(la&&(a.libraryOrder!==undefined||b.libraryOrder!==undefined))return fin(a.libraryOrder,1e9)-fin(b.libraryOrder,1e9);
    var x=bulletTitle2(a).toLowerCase(),y=bulletTitle2(b).toLowerCase();return x<y?-1:x>y?1:0;});
}
function newBullet(){return {id:G().uuid(),name:'',maker:'',caliber:'',diameterIn:0.308,weightGr:175,dragModel:'G7',bcPoints:[],seasonalBcPoints:{},dragCurve:[],source:'user'};}
function snap(){return JSON.stringify(Array.prototype.slice.call(arguments));}
// Undo for deletes: restore the records and drop their tombstones.
function undoable(msg,recs){
  snack(msg,'Undo',function(){
    var ids=recs.map(function(r){return r[1].id;});
    var L=G().lib();L.deleted=(L.deleted||[]).filter(function(t){return ids.indexOf(t.id)<0;});
    recs.forEach(function(r){G().saveRecord(r[0],r[1]);});
    rerender();
  });
}
function editorBack(dirty){return function(){blurAll();if(dirty())confirmBox('Discard changes?','You have changes that are not saved.','Discard',back,'Keep editing');else back();};}

// ============================================================================
// MY GEAR (Skyward MyGearActivity)
// ============================================================================
function GearScreen(){
  var me={tab:'gear',onGear:true};
  me.render=function(col){
    toolbar({back:toWindLab,backLabel:'Back to Wind Lab',title:'My gear',actions:[{icon:'sync',label:'Sync with base station',fn:syncNow}]});
    bottomNav(TABS,'gear',goTab);
    var f=new Form(col),rs=rifles(),c=sel(),selR=c.rifle?c.rifle.id:null,selL=c.load?c.load.id:null;
    var bw=el('div','sk-bwrap');var ab=el('button','sk-btn');add(ab,icon('add'),doc.createTextNode('Add Firearm'));on(ab,'click',function(){push(RifleEditScreen(null));});bw.appendChild(ab);f.add(bw);
    f.header('Firearms');
    if(!rs.length)f.note('You have no firearms yet. Tap "Add Firearm" to set up your first one, then add the ammo you shoot in it.');
    rs.forEach(function(r){firearmCard(f,r,loadsOf(r.id),r.id===selR,selL);});
    f.header('My bullets');
    var mine=G().bullets().filter(function(b){return b.source!=='library';}).length,libn=G().bullets().length-mine;
    f.group(function(g){
      g.nav('My bullets',''+mine,'Bullets you entered yourself, with their BCs',function(){push(BulletListScreen(2));});
      g.nav('Bullet library',''+libn,libn?'Bullets from an imported library (read-only)':'Import a Skyward bullet library in More',function(){push(BulletListScreen(1));});
    });
    f.header('Base station');
    var m=G().meta()||{};
    f.group(function(g){
      g.info('Gear sync',m.status||'not synced yet',m.err?m.err:(m.lastSync?'Last synced '+new Date(m.lastSync).toLocaleTimeString():'Your gear is kept on this phone and on the base station'));
      g.nav('Sync now','',  'Share gear with the other phones and the Skyward app',syncNow);
    });
    f.note('A tick marks what the Calculator is using. Tap a firearm or ammo to change it; ⋮ for more.');
  };
  function syncNow(){busy(true);Promise.resolve(G().sync()).then(function(){busy(false);var m=G().meta()||{};snack(m.err?('Not synced: '+m.err):'Synced with the base station');rerender();});}
  function firearmCard(f,r,loads,selected,selL){
    var card=el('div','sk-card');f.add(card);
    var shots=(G().lib().shots||[]).filter(function(x){return x.rifleId===r.id;});
    card.appendChild(itemRow(r.name,rifleSub(r)+' · '+loads.length+' ammo',selected,true,function(){push(RifleEditScreen(r.id));},[
      ['Edit',function(){push(RifleEditScreen(r.id));}],
      ['Use on the Calculator',function(){useRifle(r.id);recentUse(r.id,null);goTab('calc');}],
      ['Add Ammo',function(){push(LoadEditScreen(null,r.id));}],
      ['Round count',function(){push(ShotScreen(r.id));}],
      ['Duplicate',function(){dupRifle(r);}],
      ['Delete',function(){delRifle(r);}]]));
    var rr=el('div','sk-trow');rr.style.padding='0 12px 10px 58px';rr.style.boxSizing='border-box';
    var st=el('div','',shotSummary(shots)||'No rounds logged');st.style.flex='1';st.style.fontSize='15px';st.style.color='var(--on-var)';
    var cb=el('button','sk-obtn','Count shots');cb.style.minHeight='40px';cb.style.fontSize='16px';
    on(cb,'click',function(ev){ev.stopPropagation();push(ShotScreen(r.id));});on(rr,'click',function(){push(ShotScreen(r.id));});
    add(rr,st,cb);card.appendChild(rr);
    loads.forEach(function(l){
      card.appendChild(divider(40));
      card.appendChild(itemRow(l.name,loadSub(l,G().bullet(l.bulletId)),selected&&l.id===selL,false,function(){push(LoadEditScreen(l.id,r.id));},[
        ['Edit',function(){push(LoadEditScreen(l.id,r.id));}],
        ['Use on the Calculator',function(){useLoad(l.id);goTab('calc');}],
        ['Duplicate',function(){var cp=JSON.parse(JSON.stringify(l));cp.id=G().uuid();cp.name=l.name+' copy';delete cp.externalRef;G().saveRecord('load',cp);rerender();}],
        ['Delete',function(){delLoad(l);}]],24));
    });
    card.appendChild(divider(40));
    var a=actRow('Add Ammo',function(){push(LoadEditScreen(null,r.id));});a.style.paddingLeft='40px';a.setAttribute('aria-label','Add ammo for '+r.name);card.appendChild(a);
  }
  function divider(ind){var d=el('div','sk-div');d.style.marginLeft=ind+'px';d.style.marginRight='16px';return d;}
  function dupRifle(r){var cp=JSON.parse(JSON.stringify(r));cp.id=G().uuid();cp.name=r.name+' copy';delete cp.externalRef;G().saveRecord('rifle',cp);
    loadsOf(r.id).forEach(function(l){var x=JSON.parse(JSON.stringify(l));x.id=G().uuid();x.rifleId=cp.id;delete x.externalRef;G().saveRecord('load',x);});rerender();}
  function delRifle(r){
    var recs=[['rifle',JSON.parse(JSON.stringify(r))]].concat(loadsOf(r.id).map(function(l){return ['load',JSON.parse(JSON.stringify(l))];}));
    var wasActive=loadsOf(r.id).some(function(l){return l.id===activeId();});
    G().removeRecord('rifle',r.id);if(wasActive)useLoad('');
    rerender();undoable('Firearm deleted',recs);
  }
  function delLoad(l){
    var rec=JSON.parse(JSON.stringify(l));G().removeRecord('load',l.id);if(activeId()===l.id)useLoad('');
    rerender();undoable('Ammo deleted',[['load',rec]]);
  }
  return me;
}
function shotSummary(list){
  if(!list.length)return null;var tot=0,last=list[0];list.forEach(function(x){tot+=fin(x.count,0);if(x.recordedAt>last.recordedAt)last=x;});
  return tot.toLocaleString('en-US')+' rounds · last '+(last.count>=0?'+':'−')+Math.abs(last.count).toLocaleString('en-US')+', '+new Date(last.recordedAt).toLocaleString([], {dateStyle:'medium',timeStyle:'short'});
}
// Round count (Skyward ShotCounterActivity, short form)
function ShotScreen(rid){
  var me={add:null,note:''};
  me.render=function(col){
    var r=G().rifle(rid);if(!r){back();return;}
    toolbar({back:back,title:'Round count'});bottomNav(null);
    var list=(G().lib().shots||[]).filter(function(x){return x.rifleId===rid;}).sort(function(a,b){return b.recordedAt-a.recordedAt;});
    var tot=0;list.forEach(function(x){tot+=fin(x.count,0);});
    var f=new Form(col);
    f.header(r.name);
    f.group(function(g){g.info('Rounds fired',tot.toLocaleString('en-US'),list.length?shotSummary(list).replace(/^[^·]*· /,'Last '):'Log the rounds you fire, with the date and time');});
    f.header('Add rounds');
    f.group(function(g){
      g.number('Rounds',Q.PLAIN,me.add,function(v){me.add=v;},{decimals:0,signed:true,unit:'',hint:'e.g. 20',sub:'− to correct a mistake'});
      g.text('Note',me.note,function(t){me.note=t;},{hint:'optional'});
    });
    f.button('Add',function(){blurAll();var n=Math.round(fin(me.add,0));if(!n)return alertBox('Rounds','Enter how many rounds you fired.');
      G().saveRecord('shot',{rifleId:rid,count:n,recordedAt:Date.now(),note:(me.note||'').trim()});me.add=null;me.note='';rerender();});
    if(list.length){
      f.header('History');
      f.group(function(g){list.forEach(function(x){
        var row=g.row();label(row,(x.count>=0?'+':'−')+Math.abs(x.count).toLocaleString('en-US')+' rounds',new Date(x.recordedAt).toLocaleString([], {dateStyle:'medium',timeStyle:'short'})+(x.note?' · '+x.note:''));
        var d=el('button','sk-x');d.setAttribute('aria-label','Delete');d.appendChild(icon('close'));row.appendChild(d);
        on(d,'click',function(){var rec=JSON.parse(JSON.stringify(x));G().removeRecord('shot',x.id);rerender();undoable('Entry deleted',[['shot',rec]]);});
      });});
    }
  };
  return me;
}

// ============================================================================
// EDIT FIREARM (Skyward RifleEditActivity)
// ============================================================================
var ANG_UNITS=['MIL','MOA','IPHY'],ANG_NAMES=['MIL','MOA','IPHY (in per 100 yd)'];
function defaultClick(u,cur){if(u==='MIL')return [0.25,0.125,0.5].indexOf(cur)>=0?0.1:cur;return [0.1,0.05].indexOf(cur)>=0?0.25:cur;}
function RifleEditScreen(id){
  var ex=id?G().rifle(id):null,isNew=!ex;
  var d=ex?JSON.parse(JSON.stringify(ex)):{id:G().uuid(),name:'',twistIn:8,twistDirection:'RIGHT',sightHeightIn:1.75,sightOffsetIn:0,
    elevationUnit:'MIL',elevationClick:0.1,windageUnit:'MIL',windageClick:0.1,elevationCorrection:1,windageCorrection:1,cartridge:'',notes:''};
  ['elevationUnit','windageUnit'].forEach(function(k){if(!d[k])d[k]='MIL';});
  ['elevationClick','windageClick'].forEach(function(k){if(!(d[k]>0))d[k]=0.1;});
  ['elevationCorrection','windageCorrection'].forEach(function(k){if(!(d[k]>0))d[k]=1;});
  var initial=JSON.stringify(d);
  var me={};
  me.render=function(col){
    toolbar({back:editorBack(function(){return JSON.stringify(d)!==initial;}),backLabel:'Back',title:isNew?'Add Firearm':'Edit Firearm',actions:[{icon:'check',label:'Save',fn:save}]});
    bottomNav(null);
    var f=new Form(col),ec,wc;
    f.header('Name');
    f.group(function(g){g.text(null,d.name,function(t){d.name=t;},{hint:'Firearm name'});});
    f.header('Firearm Data');
    f.group(function(g){
      g.number('Barrel Twist',Q.BULLET_LENGTH,fin(d.twistIn,null),function(v){if(v!==null&&v>0)d.twistIn=v;},{decimals:2,sub:'Inches for one full turn of the rifling, e.g. 8'});
      g.choice('Twist Direction',['Right','Left'],d.twistDirection==='LEFT'?1:0,null,function(i){d.twistDirection=i===1?'LEFT':'RIGHT';});
      g.number('Sight Height',Q.LENGTH,fin(d.sightHeightIn,0),function(v){if(v!==null&&v>=0)d.sightHeightIn=v;},{sub:'Centre of the scope above the centre of the barrel'});
      g.number('Sight Offset',Q.LENGTH,fin(d.sightOffsetIn,0),function(v){if(v!==null)d.sightOffsetIn=v;},{signed:true,sub:'Usually 0. + when the scope sits right of the barrel'});
      g.number('Barrel Length',Q.BULLET_LENGTH,d.barrelLengthIn>0?d.barrelLengthIn:null,function(v){d.barrelLengthIn=(v!==null&&v>0)?v:null;},{decimals:1,hint:'optional'});
      g.text('Cartridge',d.cartridge||'',function(t){d.cartridge=t.trim();},{hint:'optional, e.g. 6.5 Creedmoor'});
    });
    f.header('Scope Adjustments');
    f.group(function(g){
      g.choice('Elevation Unit',ANG_NAMES,Math.max(0,ANG_UNITS.indexOf(d.elevationUnit)),null,function(i){d.elevationUnit=ANG_UNITS[i];d.elevationClick=defaultClick(d.elevationUnit,d.elevationClick);ec.set(d.elevationClick);});
      ec=g.number('Elevation Click',Q.PLAIN,d.elevationClick,function(v){if(v!==null&&v>0)d.elevationClick=v;},{decimals:4,unit:'',sub:'turret value per click, e.g. 0.1 MIL, 0.25 MOA'});
      g.choice('Windage Unit',ANG_NAMES,Math.max(0,ANG_UNITS.indexOf(d.windageUnit)),null,function(i){d.windageUnit=ANG_UNITS[i];d.windageClick=defaultClick(d.windageUnit,d.windageClick);wc.set(d.windageClick);});
      wc=g.number('Windage Click',Q.PLAIN,d.windageClick,function(v){if(v!==null&&v>0)d.windageClick=v;},{decimals:4,unit:''});
    });
    f.header('Scope correction (optional)');
    f.group(function(g){
      g.number('Elevation Correction',Q.PLAIN,d.elevationCorrection,function(v){if(v!==null&&v>0)d.elevationCorrection=v;},{decimals:4,unit:''});
      g.number('Windage Correction',Q.PLAIN,d.windageCorrection,function(v){if(v!==null&&v>0)d.windageCorrection=v;},{decimals:4,unit:''});
    });
    f.note('Correction factors multiply the calculated adjustment (dial value and clicks) to compensate for scope inaccuracies. Use 1.0 unless you know your scope tracks differently.');
    if(!isNew){
      var shots=(G().lib().shots||[]).filter(function(x){return x.rifleId===d.id;}),tot=0;shots.forEach(function(x){tot+=fin(x.count,0);});
      f.header('Round count');
      f.group(function(g){g.nav('Round count',tot.toLocaleString('en-US'),shots.length?'Last entry '+new Date(Math.max.apply(null,shots.map(function(x){return x.recordedAt;}))).toLocaleDateString():'Log the rounds you fire, with the date and time',function(){push(ShotScreen(d.id));});});
    }
    f.header('Notes');
    f.group(function(g){g.text(null,d.notes||'',function(t){d.notes=t;},{hint:'Notes',multiline:true});});
  };
  function save(){
    blurAll();d.name=(d.name||'').trim();
    if(!d.name)return alertBox('Missing name','Give the firearm a profile name.');
    if(!(d.twistIn>0))return alertBox('Barrel twist','Enter the barrel twist (inches per turn).');
    if(!(d.elevationClick>0&&d.windageClick>0))return alertBox('Click value','Click values must be greater than 0.');
    G().saveRecord('rifle',d);initial=JSON.stringify(d);
    if(!sel().rifle||isNew&&!activeId())lsSet('skyRifleId',d.id);
    refreshActive();back();
  }
  return me;
}

// ============================================================================
// EDIT AMMO (Skyward LoadEditActivity)
// ============================================================================
function LoadEditScreen(id,rifleId){
  var ex=id?G().load(id):null,isNew=!ex,l,b,bulletIsNew,src,model,rows;
  if(ex){
    l=JSON.parse(JSON.stringify(ex));rifleId=l.rifleId;
    var b0=G().bullet(l.bulletId);
    if(b0){b=JSON.parse(JSON.stringify(b0));bulletIsNew=false;}else{b=newBullet();b.id=l.bulletId||b.id;bulletIsNew=true;}
    src=b0?bcDetect(l,b):'CUSTOM';
    model=(l.dragModelOverride&&l.dragModelOverride!=='CUSTOM')?l.dragModelOverride:(b.dragModel&&b.dragModel!=='CUSTOM'?b.dragModel:'G7');
    rows=bcDrafts((l.bcPointsOverride&&l.bcPointsOverride.length)?l.bcPointsOverride:b.bcPoints);
  }else{
    b=newBullet();bulletIsNew=true;
    l={id:G().uuid(),rifleId:rifleId,name:'',bulletId:b.id,muzzleVelocityFps:2800,zeroRangeYd:100,zeroHeightIn:0,zeroOffsetIn:0,
       powderTempF:59,velocityPerDegFps:0,dragScaleFactor:1,notes:'',zeroConfirmed:true};
    src='BULLET';model='G7';rows=[{bc:null,v:0}];
  }
  if(!(fin(l.dragScaleFactor,0)>0))l.dragScaleFactor=1;
  function snapshot(){return JSON.stringify([l,b,src,model,rows]);}
  var initial=snapshot(),me={},tnf=null;
  function isLib(){return b.source==='library';}
  function editable(){return src==='CUSTOM'||(src==='BULLET'&&!isLib());}
  function shown(s){return DS_SEASON[s]?((b.seasonalBcPoints||{})[DS_SEASON[s]]||[]):(b.bcPoints||[]);}
  function resetDrag(s){src=s;model=(b.dragModel&&b.dragModel!=='CUSTOM')?b.dragModel:'G7';rows=bcDrafts(shown(s));}
  me.render=function(col){
    toolbar({back:editorBack(function(){return snapshot()!==initial;}),backLabel:'Back',title:isNew?'Add Ammo':'Edit Ammo',actions:[{icon:'check',label:'Save',fn:save}]});
    bottomNav(null);
    var f=new Form(col);
    f.header('Name');
    f.group(function(g){g.text(null,l.name,function(t){l.name=t;},{hint:'Ammo Name'});});
    f.header('Bullet');
    f.group(function(g){
      var own=bulletIsNew&&!(b.name||'').trim();
      g.nav('Bullet',own?'My own BC':bulletTitle2(b),isLib()?bulletSource(b)+'. Tap to search for another bullet':own?'Enter the BC below, or tap to search the bullet library and your bullets':'My bullet. Tap to search for another bullet',changeBullet);
      if(isLib()){
        g.info('Diameter',Num.trim(fin(b.diameterIn,0),3)+' in');g.info('Weight',Num.trim(fin(b.weightGr,0),1)+' gr');
        g.info('Length',b.lengthIn>0?Num.trim(b.lengthIn,3)+' in':'estimated');
      }else{
        g.number('Bullet Diameter',Q.BULLET_LENGTH,fin(b.diameterIn,null),function(v){if(v!==null&&v>0)b.diameterIn=v;});
        g.number('Bullet Weight',Q.WEIGHT,fin(b.weightGr,null),function(v){if(v!==null&&v>0)b.weightGr=v;});
        g.number('Bullet Length',Q.BULLET_LENGTH,b.lengthIn>0?b.lengthIn:null,function(v){b.lengthIn=(v!==null&&v>0)?v:null;},{hint:'optional',sub:'for stability / spin drift'});
        g.number('Nose Length',Q.BULLET_LENGTH,b.noseLengthIn>0?b.noseLengthIn:null,function(v){b.noseLengthIn=(v!==null&&v>0)?v:null;},{hint:'optional',sub:'only for aerodynamic jump'});
      }
    });
    f.header('Speed and drag');
    f.group(function(g){
      var need=!(fin(l.muzzleVelocityFps,0)>=300);
      g.number('Muzzle Velocity',Q.VELOCITY,fin(l.muzzleVelocityFps,0)>0?l.muzzleVelocityFps:null,function(v){if(v!==null&&v>0){l.muzzleVelocityFps=v;l.muzzleVelocitySource='';updTnf();}},
        {hint:'needed',sub:need?'Needed: enter the bullet speed leaving the barrel (chronograph or ammo box)':(l.muzzleVelocitySource?'From '+l.muzzleVelocitySource:'Bullet speed leaving the barrel (chronograph or ammo box)')});
      var ch=bcChoices(b);
      g.choice('Drag data',ch.map(function(k){return DS[k];}),Math.max(0,ch.indexOf(src)),null,function(i){
        var s=ch[i];
        if(s==='CUSTOM'&&src!=='CUSTOM'){model=((src==='BULLET'?b.dragModel:'G7')!=='CUSTOM'&&(src==='BULLET'?b.dragModel:'G7'))||'G7';
          rows=bcDrafts((src==='CURVE'||src==='AUTO')?b.bcPoints:shown(src));src=s;}
        else resetDrag(s);
        rerender();});
      if(editable())g.choice('Drag Model',['G1','G7'],model==='G1'?0:1,null,function(i){model=i===0?'G1':'G7';updTnf();});
      g.number('Drag Scale Factor',Q.PLAIN,fin(l.dragScaleFactor,1),function(v){if(v!==null&&v>0){l.dragScaleFactor=v;}},{decimals:4,unit:'',sub:'1.0 = as measured. Calibrate can set this for you'});
      tnf=g.info('TNF','—','Muzzle velocity × G7 BC');
    });
    if(editable()){
      f.header('Ballistic Coefficient');
      f.group(function(g){g.custom(bcRowsView(rows,function(){rerender();}));});
      f.note('Enter the ballistic coefficient (BC) from the bullet box. If the maker lists several BCs, add a row for each with the lowest speed it applies to. One row must have a lowest speed of 0.');
    }else if(src==='CURVE'&&hasCurve(b)){
      f.header('Measured Drag Curve');
      f.group(function(g){g.custom(curveView(b.dragCurve));});
      f.note('The measured curve is the most accurate drag model for this bullet.');
    }else if(src==='AUTO'){
      f.header('Seasonal BC rows');
      f.group(function(g){['cold','mild','hot'].forEach(function(s){var r=(b.seasonalBcPoints||{})[s];if(r&&r.length)g.info(s.charAt(0).toUpperCase()+s.slice(1)+' (G7)',bcRowsText(r));});});
      f.note('Picked at calculation time from the shooting temperature: cold below 30°F, hot above 90°F, mild in between.');
    }else{
      f.header('Ballistic Coefficient');
      f.group(function(g){g.info('G7 rows',bcRowsText(shown(src)));});
    }
    f.header('Powder temperature (optional)');
    f.group(function(g){
      g.number('Powder Temperature',Q.TEMPERATURE,fin(l.powderTempF,59),function(v){if(v!==null)l.powderTempF=v;},{signed:true});
      g.number('Speed change per degree',Q.VEL_PER_DEG,fin(l.velocityPerDegFps,0)!==0?l.velocityPerDegFps:null,function(v){l.velocityPerDegFps=v===null?0:v;},{signed:true,hint:'0'});
    });
    f.note('Optional. Your ammo’s powder temperature when the muzzle velocity was measured, and the change in velocity (+/−) per degree.');
    f.header('Zero (sight-in)');
    if(l.zeroConfirmed===false)f.note('Zero not confirmed. Check it against how the firearm is sighted in, then tap Save.');
    f.group(function(g){
      g.number('Zero Range',Q.DISTANCE,fin(l.zeroRangeYd,100),function(v){if(v!==null&&v>0)l.zeroRangeYd=v;},{sub:'The distance the firearm is sighted in at'});
      g.number('Zero Height',Q.LENGTH,fin(l.zeroHeightIn,0),function(v){l.zeroHeightIn=v===null?0:v;},{signed:true,sub:'+ impact above aim. Same as the Wind Lab group offset'});
      g.number('Zero Offset',Q.LENGTH,fin(l.zeroOffsetIn,0),function(v){l.zeroOffsetIn=v===null?0:v;},{signed:true,sub:'+ impact right of aim'});
      g.sw('Zero was shot in different weather',!!l.zeroAtmosphere,'Turn on to enter the weather on the day you zeroed',function(c){
        l.zeroAtmosphere=c?(l.zeroAtmosphere||G().atmoOf(params())):null;rerender();});
      var za=l.zeroAtmosphere;
      if(za){
        g.number('Altitude',Q.ALTITUDE,fin(za.altitudeFt,0),function(v){if(v!==null&&v>-2000&&v<30000)za.altitudeFt=v;},{signed:true});
        g.number('Pressure',Q.PRESSURE,fin(za.pressureInHg,29.92),function(v){if(v!==null&&v>10&&v<35)za.pressureInHg=v;});
        g.sw('Pressure was measured here',za.pressureIsAbsolute!==false,'On: station (absolute) pressure. Off: sea-level pressure from a forecast',function(c){za.pressureIsAbsolute=c;});
        g.number('Temperature',Q.TEMPERATURE,fin(za.temperatureF,59),function(v){if(v!==null&&v>-80&&v<160)za.temperatureF=v;},{signed:true});
        g.number('Humidity',Q.HUMIDITY,fin(za.humidityPct,0),function(v){if(v!==null&&v>=0&&v<=100)za.humidityPct=v;});
      }
    });
    f.header('Notes');
    f.group(function(g){g.text(null,l.notes||'',function(t){l.notes=t;},{hint:'Notes',multiline:true});});
    if(!isNew){
      f.add(el('div','sk-gap'));
      var del=f.button('Delete this ammo',function(){confirmBox('Delete ammo','Delete "'+l.name+'"?','Delete',function(){
        var rec=JSON.parse(JSON.stringify(G().load(l.id)||l));G().removeRecord('load',l.id);if(activeId()===l.id)useLoad('');initial=snapshot();back();
        setTimeout(function(){undoable('Ammo deleted',[['load',rec]]);},60);});},true);
      del.style.color='var(--error)';
    }
    updTnf();
    // keep the TNF line fresh while typing
    var t=setInterval(function(){if(!tnf||!doc.body.contains(tnf)){clearInterval(t);return;}updTnf();},700);
  };
  function draftFlight(){
    if(!(fin(l.muzzleVelocityFps,0)>=300))return null;
    var bb=JSON.parse(JSON.stringify(b)),fr=[];
    if(editable()){var r=bcFinish(rows);if(!r.rows)return null;fr=r.rows;}
    if(!isLib()&&src==='BULLET'){bb.dragModel=model;bb.bcPoints=fr;}
    if(!isLib()&&src==='CUSTOM'&&!(bb.bcPoints||[]).length&&!hasCurve(bb)){bb.dragModel=model;bb.bcPoints=fr;}
    return {l:bcApply(l,bb,src,model,fr),b:bb};
  }
  function updTnf(){
    if(!tnf)return;var fl=draftFlight(),v=null;
    if(fl){
      var mdl=fl.l.dragModelOverride||fl.b.dragModel||'G7',rs=(fl.l.bcPointsOverride&&fl.l.bcPointsOverride.length)?fl.l.bcPointsOverride:(fl.l.bcSeason?((fl.b.seasonalBcPoints||{})[fl.l.bcSeason==='auto'?'mild':fl.l.bcSeason]||[]):fl.b.bcPoints||[]);
      var top=rs.slice().sort(function(a,c){return c.minVelFps-a.minVelFps;})[0];
      if(mdl==='G7'&&top)v=fl.l.muzzleVelocityFps*top.bc;
    }
    var t=v===null?'—':Num.fixed(v,0);if(tnf.textContent!==t)tnf.textContent=t;
    if(tnf.label&&tnf.label.sub)tnf.label.sub.textContent=v!==null?'Muzzle velocity × G7 BC (top band)':(fl?'Muzzle velocity × G7 BC: shown for G7 BCs':'Muzzle velocity × G7 BC: enter both to see it');
  }
  function changeBullet(){
    blurAll();
    push(BulletListScreen(0,function(pickId){
      if(pickId===null){ // enter my own BC
        if(bulletIsNew&&!(b.name||'').trim())return;
        if(isLib()){b=JSON.parse(JSON.stringify(b));b.id=G().uuid();b.source='user';delete b.libraryOrder;b.seasonalBcPoints={};b.dragCurve=[];b.dragModel='G7';b.name='';}
        else b=newBullet();
        bulletIsNew=true;l.bulletId=b.id;src='BULLET';model='G7';rows=bcDrafts(b.bcPoints);return;
      }
      var nb=G().bullet(pickId);if(!nb)return;
      b=JSON.parse(JSON.stringify(nb));bulletIsNew=false;l.bulletId=b.id;
      if(!(l.name||'').trim())l.name=bulletTitle2(b);
      resetDrag(bcDefault(b));
    }));
  }
  function save(){
    blurAll();
    var L=JSON.parse(JSON.stringify(l)),B=JSON.parse(JSON.stringify(b));L.name=(L.name||'').trim();
    if(!L.name)return alertBox('Missing name','Give the ammo a profile name.');
    if(!(L.muzzleVelocityFps>=300&&L.muzzleVelocityFps<=6000))return alertBox('Muzzle velocity','Enter a muzzle velocity between 300 and 6000 fps.');
    if(!(B.diameterIn>=0.1&&B.diameterIn<=1))return alertBox('Bullet diameter','Enter the bullet diameter in inches (e.g. 0.264).');
    if(!(B.weightGr>0))return alertBox('Bullet weight','Enter the bullet weight in grains.');
    if(!(L.zeroRangeYd>0))return alertBox('Zero range','Enter the zero range.');
    var fr=[];
    if(editable()){var r=bcFinish(rows);if(!r.rows)return alertBox('Ballistic coefficient',r.err||'Check the BC rows.');fr=r.rows;}
    if(src==='CURVE'&&!hasCurve(B))return alertBox('Drag','This bullet has no measured drag curve.');
    if(!isLib()){
      if(src==='BULLET'){B.dragModel=model;B.bcPoints=fr;}
      if(!(B.name||'').trim())B.name=L.name;
      if(src==='CUSTOM'&&!(B.bcPoints||[]).length&&!hasCurve(B)){B.dragModel=model;B.bcPoints=fr;}
      var stored=G().bullet(B.id);
      if(!stored||JSON.stringify(Object.assign({},stored,{updatedAt:0}))!==JSON.stringify(Object.assign({},B,{updatedAt:0}))){
        var shared=stored&&G().lib().loads.some(function(x){return x.bulletId===B.id&&x.id!==L.id;});
        if(shared){B.id=G().uuid();B.name=B.name+' ('+L.name+')';}
        B.source='user';B.id=G().saveRecord('bullet',B);
      }
    }
    L.bulletId=B.id;L.zeroConfirmed=true;
    L=bcApply(L,B,src,model,fr);
    G().saveRecord('load',L);
    var cur=sel();
    if(!activeId()||(isNew&&cur.rifle&&cur.rifle.id===L.rifleId)){useLoad(L.id,false);}
    else refreshActive();
    initial=snapshot();l=L;b=B;back();
  }
  return me;
}
// Measured drag curve chart (Skyward DragCurveView), Cd vs Mach with the G7 reference dashed
function curveView(curve){
  var pts=curve.slice().sort(function(a,b){return a.mach-b.mach;}),NS='http://www.w3.org/2000/svg',W=320,H=200;
  var box=el('div');box.style.width='100%';
  var svg=doc.createElementNS(NS,'svg');svg.setAttribute('viewBox','0 0 '+W+' '+H);svg.style.width='100%';svg.style.height='200px';
  function mk(t,a){var e=doc.createElementNS(NS,t);for(var k in a)e.setAttribute(k,a[k]);svg.appendChild(e);return e;}
  var m0=pts[0].mach,m1=pts[pts.length-1].mach,g7=pts.map(function(p){return root.SkyEngine&&SkyEngine.lookup?SkyEngine.lookup(SkyEngine.G7,p.mach):null;});
  var cdMax=Math.max.apply(null,pts.map(function(p){return p.cd;}).concat(g7.filter(function(x){return x!==null;})))*1.1;
  var L=40,B=H-22,R=W-6,T=6;function x(m){return L+(m-m0)/Math.max(1e-6,m1-m0)*(R-L);}function y(c){return B-c/cdMax*(B-T);}
  mk('line',{x1:L,y1:B,x2:R,y2:B,stroke:'var(--on-var)'});mk('line',{x1:L,y1:T,x2:L,y2:B,stroke:'var(--on-var)'});
  [['Cd',2,T+12],[Num.trim(cdMax,2),2,T+28],['Mach '+Num.trim(m0,2),L,H-4]].forEach(function(t){var e=mk('text',{x:t[1],y:t[2],'font-size':12,fill:'var(--on-var)'});e.textContent=t[0];});
  var e2=mk('text',{x:R,y:H-4,'font-size':12,fill:'var(--on-var)','text-anchor':'end'});e2.textContent=Num.trim(m1,2);
  if(g7.every(function(v){return v!==null;}))mk('path',{d:pts.map(function(p,i){return (i?'L':'M')+x(p.mach).toFixed(1)+','+y(g7[i]).toFixed(1);}).join(''),fill:'none',stroke:'var(--on-var)','stroke-dasharray':'4 4'});
  mk('path',{d:pts.map(function(p,i){return (i?'L':'M')+x(p.mach).toFixed(1)+','+y(p.cd).toFixed(1);}).join(''),fill:'none',stroke:'var(--primary)','stroke-width':2});
  box.appendChild(svg);
  box.appendChild(el('div','sk-note','Measured Cd vs Mach ('+pts.length+' points, Mach '+Num.trim(m0,2)+'–'+Num.trim(m1,2)+')'+(g7[0]!==null?'; dashed: G7 reference':''))).style.padding='4px 0 0';
  return box;
}

// ============================================================================
// BULLETS (Skyward BulletLibraryActivity / BulletEditActivity)
// filter: 0 all, 1 library, 2 mine. onPick(id|null) = choose a bullet for ammo.
// ============================================================================
function BulletListScreen(filter,onPick){
  var me={q:'',filter:filter||0};
  me.render=function(col){
    var title=onPick?'Choose the bullet':(filter===2?'My bullets':'Bullet library');
    toolbar({back:back,title:title});bottomNav(null);
    var sb=el('div','sk-search');sb.appendChild(icon('search'));var inp=el('input');inp.type='search';inp.value=me.q;
    inp.placeholder=onPick?'Search bullets: maker, name, calibre, 140gr…':'Search bullets';sb.appendChild(inp);col.appendChild(sb);
    if(!onPick){
      var segs=el('div','sk-row');segs.style.padding='4px 10px';['All','Library','My bullets'].forEach(function(t,i){
        var bt=el('button','sk-seg'+(i===me.filter?' on':''),t);on(bt,'click',function(){me.filter=i;draw();Array.prototype.forEach.call(segs.children,function(x,j){x.className='sk-seg'+(j===i?' on':'');});});segs.appendChild(bt);});
      col.appendChild(segs);
      var bw=el('div','sk-bwrap');var ab=el('button','sk-obtn');ab.style.width='100%';add(ab,icon('add'),doc.createTextNode('Add bullet'));on(ab,'click',addBullet);bw.appendChild(ab);col.appendChild(bw);
    }
    var list=el('div');col.appendChild(list);
    if(onPick){
      var own=el('div','sk-bwrap');own.style.paddingBottom='24px';var ob=el('button','sk-btn','Enter my own BC');on(ob,'click',function(){done(null);});own.appendChild(ob);col.appendChild(own);
    }
    on(inp,'input',function(){me.q=inp.value;draw();});
    function draw(){
      list.innerHTML='';var all=bulletsOrdered();
      var items=all.filter(function(b){return (onPick||me.filter===0||(me.filter===1)===(b.source==='library'))&&bulletMatches(b,me.q);});
      if(!items.length){
        var msg=onPick?(all.length?'No bullet matches your search.\n\nTry fewer words, or tap "Enter my own BC" below.':'No bullet library imported, and no bullets of your own yet.\n\nThat is fine: tap "Enter my own BC" below and type the BC from the box.')
          :(me.filter===2&&!me.q?'Your bullets — add the bullets you shoot, with their BCs. They’re kept on this phone and on the base station.'
          :(!all.some(function(b){return b.source==='library';})&&me.filter!==2&&!me.q?'No bullet library imported yet.\n\nImport a Skyward Ballistics bullet library file in More → Import gear. You can still add your own bullets with Add bullet.'
          :'No bullets match your search.\n\nTry fewer words, or tap "Add bullet" to enter your own.'));
        list.appendChild(el('div','sk-empty',msg));return;
      }
      var shownN=0;
      items.some(function(b){
        if(shownN++>=150){list.appendChild(el('div','sk-note','Showing the first 150. Type more of the name to narrow the list.'));return true;}
        var row=itemRow(bulletTitle2(b),[b.caliber,bulletSub(b)].filter(Boolean).join(' · '),false,false,function(){if(onPick)done(b.id);else push(BulletEditScreen(b.id));});
        var tg=onPick?bulletSource(b):(b.source==='library'?null:'mine');
        if(tg){row.querySelector('.t').appendChild(el('span','sk-tag',tg));}
        row.querySelector('.ck').style.display='none';
        list.appendChild(row);return false;
      });
    }
    draw();
  };
  function done(id){var cb=onPick;back();setTimeout(function(){cb(id);if(current())rerender();},0);}
  function addBullet(){
    if(!G().bullets().length)return push(BulletEditScreen(null));
    choose('Add bullet',['Start blank','Copy from a bullet…'],-1,function(i){
      if(i===0)push(BulletEditScreen(null));
      else push(BulletListScreen(0,function(id){if(id)setTimeout(function(){push(BulletEditScreen(null,id));},150);}));
    });
  }
  return me;
}
function BulletEditScreen(id,copyFrom){
  var ex=id?G().bullet(id):null,src0=copyFrom?G().bullet(copyFrom):null,isNew=!ex;
  var b=ex?JSON.parse(JSON.stringify(ex)):(src0?Object.assign(JSON.parse(JSON.stringify(src0)),{id:G().uuid(),source:'user',name:(src0.name||'')+' copy'}):newBullet());
  if(src0){delete b.libraryOrder;delete b.collection;}
  var rows=bcDrafts(b.bcPoints),initial=JSON.stringify([b,rows]),me={};
  var readOnly=b.source==='library';
  me.render=function(col){
    var acts=[];
    if(!readOnly)acts.push({icon:'check',label:'Save',fn:save});
    acts.push({icon:'dots',label:'More',fn:function(){menu(shell.bar.lastChild,[
      ['Copy to my bullets',function(){push(BulletEditScreen(null,b.id));}]].concat(isNew?[]:[['Delete',del]]));}});
    toolbar({back:readOnly?back:editorBack(function(){return JSON.stringify([b,rows])!==initial;}),backLabel:'Back',title:readOnly?'Library bullet':(isNew?'Add bullet':'Edit bullet'),actions:acts});
    bottomNav(null);
    var f=new Form(col);
    if(readOnly){
      f.header('Bullet');
      f.group(function(g){g.info('Name',bulletTitle2(b));g.info('Maker',b.maker||'—');g.info('Caliber',b.caliber||'—');if(b.category)g.info('Type',b.category);
        g.info('Diameter',Num.trim(fin(b.diameterIn,0),3)+' in');g.info('Weight',Num.trim(fin(b.weightGr,0),1)+' gr');g.info('Length',b.lengthIn>0?Num.trim(b.lengthIn,4)+' in':'—');
        if(b.noseLengthIn>0)g.info('Nose length',Num.trim(b.noseLengthIn,4)+' in');});
      if(hasCurve(b)){f.header('Measured drag curve');f.group(function(g){g.custom(curveView(b.dragCurve));});}
      if((b.bcPoints||[]).length){f.header('All-season BC rows ('+(b.dragModel!=='CUSTOM'?(b.dragModel||'G7'):'G7')+')');f.group(function(g){g.info('BC  ≥ min vel',bcRowsText(b.bcPoints));});}
      seasons(b).forEach(function(s){f.header(s+'-weather BC rows (G7)');f.group(function(g){g.info('BC  ≥ min vel',bcRowsText(b.seasonalBcPoints[s]));});});
      if(b.notes){f.header('Notes');f.note(b.notes);}
      f.note('Library bullets are read-only. Use ⋮ → Copy to my bullets to make an editable copy.');
      return;
    }
    f.header('Bullet');
    f.group(function(g){
      g.text('Name',b.name||'',function(t){b.name=t;},{hint:'e.g. 140 ELD-M'});
      g.text('Maker',b.maker||'',function(t){b.maker=t;},{hint:'optional'});
      g.text('Caliber',b.caliber||'',function(t){b.caliber=t;},{hint:'optional, e.g. 6.5 mm'});
    });
    f.header('Dimensions');
    f.group(function(g){
      g.number('Diameter',Q.BULLET_LENGTH,b.diameterIn>0?b.diameterIn:null,function(v){if(v!==null&&v>0)b.diameterIn=v;});
      g.number('Weight',Q.WEIGHT,b.weightGr>0?b.weightGr:null,function(v){if(v!==null&&v>0)b.weightGr=v;});
      g.number('Length',Q.BULLET_LENGTH,b.lengthIn>0?b.lengthIn:null,function(v){b.lengthIn=(v!==null&&v>0)?v:null;},{hint:'optional',sub:'for stability / spin drift'});
      g.number('Nose length',Q.BULLET_LENGTH,b.noseLengthIn>0?b.noseLengthIn:null,function(v){b.noseLengthIn=(v!==null&&v>0)?v:null;},{hint:'optional',sub:'only for aerodynamic jump'});
    });
    var models=['G1','G7'].concat(hasCurve(b)?['CUSTOM']:[]);
    f.header('Drag model');
    f.group(function(g){g.choice('Drag model',models.map(function(m){return m==='CUSTOM'?'Measured drag curve':m;}),Math.max(0,models.indexOf(b.dragModel||'G7')),null,function(i){b.dragModel=models[i];rerender();});});
    if(b.dragModel==='CUSTOM'&&hasCurve(b))f.group(function(g){g.custom(curveView(b.dragCurve));});
    f.header('Ballistic coefficient (BC)');
    f.group(function(g){g.custom(bcRowsView(rows,function(){rerender();}));});
    f.note('Enter the ballistic coefficient (BC) from the bullet box. If the maker lists several BCs, add a row for each with the lowest speed it applies to. One row must have a lowest speed of 0.'+(b.dragModel==='CUSTOM'?' Optional with a measured curve.':''));
    f.header('Notes');
    f.group(function(g){g.text(null,b.notes||'',function(t){b.notes=t;},{hint:'Notes',multiline:true});});
  };
  function save(){
    blurAll();b.name=(b.name||'').trim();
    if(!b.name)return alertBox('Missing name','Give the bullet a name.');
    if(!(b.diameterIn>=0.1&&b.diameterIn<=1))return alertBox('Diameter','Enter the diameter in inches, between 0.1 and 1.0.');
    if(!(b.weightGr>0&&b.weightGr<2000))return alertBox('Weight','Enter the weight in grains.');
    var fil=rows.filter(function(r){return r.bc!==null||r.v!==null;});
    if(fil.length||b.dragModel!=='CUSTOM'){var r=bcFinish(rows);if(!r.rows)return alertBox('Ballistic coefficient',r.err||'Check the BC rows.');b.bcPoints=r.rows;}
    else b.bcPoints=[];
    b.source='user';G().saveRecord('bullet',b);initial=JSON.stringify([b,rows]);refreshActive();back();
  }
  function del(){
    if(G().lib().loads.some(function(l){return l.bulletId===b.id;}))return alertBox('Bullet in use','Some ammo still uses this bullet. Change or delete that ammo first.');
    confirmBox('Delete bullet','Delete "'+bulletTitle2(b)+'"?','Delete',function(){var rec=JSON.parse(JSON.stringify(G().bullet(b.id)||b));G().removeRecord('bullet',b.id);initial=JSON.stringify([b,rows]);back();
      setTimeout(function(){undoable('Bullet deleted',[['bullet',rec]]);},60);});
  }
  return me;
}


// ============================================================================
// ANALYSIS (Skyward AnalysisActivity + HitSimActivity, analysis/HitSim.kt)
// ============================================================================
function AnalysisScreen(){
  var me={tab:'analysis'};
  me.render=function(col){
    toolbar({back:toWindLab,backLabel:'Back to Wind Lab',title:'Analysis'});
    bottomNav(TABS,'analysis',goTab);
    var f=new Form(col);
    f.add(el('div','sk-gap'));
    f.group(function(g){
      g.nav('Hit probability','','Your chance of hitting a target at a distance, given how well you know the range, the wind and your velocity, and how well your rifle groups',
        function(){push(HitSimScreen());});
      g.nav('Wind sandbox','','Wind Lab’s what-if tool: how much each stretch of the bullet’s path adds to the wind drift',
        function(){closeSky();if(root.setSection)try{root.setSection('sandbox');}catch(e){}});
    });
    f.note('The sandbox opens in Wind Lab. Come back with the Solver tab.');
  };
  return me;
}

var HS={ES5:3.077,IN_PER_MIL_PER_YD:0.036,MV:10,BC:0.02,RANGE:5,CROSS:10,HEAD:10};
var HS_DEF={useSaved:true,loadId:null,diameterIn:0.264,weightGr:140,bc:0.310,g1:false,mvFps:2750,sightHeightIn:1.75,zeroYd:100,twistIn:8,
  rangeYd:600,targetWidthIn:12,targetHeightIn:12,round:false,windMph:8,windFromDeg:90,mvSdFps:10,bcSdPct:1.5,rangeSdYd:3,windSpeedSdMph:1.5,windDirSdDeg:10,groupMoa:0.75,aimSdMoa:0.2,shots:5000};
function hsState(){var s=lsJson('skyHitSim',{}),o={};for(var k in HS_DEF)o[k]=(s&&s[k]!==undefined&&s[k]!==null)?s[k]:HS_DEF[k];return o;}
function rng(seed){var a=seed>>>0,spare=null;
  function u(){a=(a+0x6D2B79F5)>>>0;var t=a;t=Math.imul(t^(t>>>15),t|1);t^=t+Math.imul(t^(t>>>7),t|61);return ((t^(t>>>14))>>>0)/4294967296;}
  return function(){if(spare!==null){var s=spare;spare=null;return s;}var x,y,r;do{x=2*u()-1;y=2*u()-1;r=x*x+y*y;}while(r>=1||r===0);var m=Math.sqrt(-2*Math.log(r)/r);spare=y*m;return x*m;};}
var HitSim={
  sensitivities:function(rifle,load,bullet,shot,windMph,windFromDeg,ranges){
    ranges=ranges.filter(function(r){return r>HS.RANGE;}).sort(function(a,b){return a-b;});if(!ranges.length)return [];
    var all=[];ranges.forEach(function(r){[r-HS.RANGE,r,r+HS.RANGE].forEach(function(x){if(all.indexOf(x)<0)all.push(x);});});all.sort(function(a,b){return a-b;});
    var max=all[all.length-1];
    var base=Object.assign({},shot,{distanceYd:max,windSpeed1Mph:windMph,windAngleDeg:windFromDeg,windSpeed2Mph:0,targetSpeedMph:0});
    function run(s,dv,bc){return SkyEngine.solve(rifle,load,bullet,s,{maxRangeYd:max,stepYd:max,extraRangesYd:all,mvShiftFps:dv||0,bcFactor:bc||1});}
    var s0=run(base),sMv=run(base,HS.MV),sBc=run(base,0,1+HS.BC);
    var wa=windFromDeg*Math.PI/180;
    function windShot(h,c){return Object.assign({},base,{windSpeed1Mph:Math.sqrt(h*h+c*c),windAngleDeg:Math.atan2(c,h)*180/Math.PI});}
    var sX=run(windShot(windMph*Math.cos(wa),windMph*Math.sin(wa)+HS.CROSS));
    var sH=run(windShot(windMph*Math.cos(wa)+HS.HEAD,windMph*Math.sin(wa)));
    return ranges.map(function(r){
      var o=s0.at(r);function dv(p){return o.comeUpMil-p.at(r).comeUpMil;}function dh(p){return o.windageMil-p.at(r).windageMil;}
      var lo=s0.at(r-HS.RANGE),hi=s0.at(r+HS.RANGE);
      return {rangeYd:r,vPerFps:dv(sMv)/HS.MV,hPerFps:dh(sMv)/HS.MV,vPerBcPct:dv(sBc)/(HS.BC*100),hPerBcPct:dh(sBc)/(HS.BC*100),
        vPerYd:(hi.comeUpMil-lo.comeUpMil)/(2*HS.RANGE),hPerYd:(hi.windageMil-lo.windageMil)/(2*HS.RANGE),
        vPerCrossMph:dv(sX)/HS.CROSS,hPerCrossMph:dh(sX)/HS.CROSS,vPerHeadMph:dv(sH)/HS.HEAD,hPerHeadMph:dh(sH)/HS.HEAD};
    });
  },
  rms:function(sd,f){if(sd<=0)return Math.abs(f(0));var n=240,sw=0,s=0;for(var j=0;j<=n;j++){var z=-6+12*j/n,w=Math.exp(-z*z/2)*((j===0||j===n)?0.5:1),v=f(z*sd);sw+=w;s+=w*v*v;}return Math.sqrt(s/sw);},
  causes:function(s,i){
    var k=s.rangeYd*HS.IN_PER_MIL_PER_YD,wa=i.windFromDeg*Math.PI/180;
    function wV(c,h){return s.vPerCrossMph*c+s.vPerHeadMph*h;}function wH(c,h){return s.hPerCrossMph*c+s.hPerHeadMph*h;}
    function C(name,v,h){return {name:name,vIn:v,hIn:h,note:'',total:Math.sqrt(v*v+h*h)};}
    var sw=i.windSpeedSdMph,sd=i.windDirSdDeg*Math.PI/180;
    function dC(d){return i.windMph*(Math.sin(wa+d)-Math.sin(wa));}function dH(d){return i.windMph*(Math.cos(wa+d)-Math.cos(wa));}
    var g=(i.groupMoa/HS.ES5)/MOA_PER_MIL*k,aim=i.aimSdMoa/MOA_PER_MIL*k;
    var all=[];
    if(sw>0)all.push(C('Wind speed',Math.abs(wV(sw*Math.sin(wa),sw*Math.cos(wa)))*k,Math.abs(wH(sw*Math.sin(wa),sw*Math.cos(wa)))*k));
    if(i.windDirSdDeg>0)all.push(C('Wind direction',HitSim.rms(sd,function(d){return wV(dC(d),dH(d));})*k,HitSim.rms(sd,function(d){return wH(dC(d),dH(d));})*k));
    if(i.rangeSdYd>0)all.push(C('Range',Math.abs(s.vPerYd)*i.rangeSdYd*k,Math.abs(s.hPerYd)*i.rangeSdYd*k));
    if(i.mvSdFps>0)all.push(C('Muzzle velocity',Math.abs(s.vPerFps)*i.mvSdFps*k,Math.abs(s.hPerFps)*i.mvSdFps*k));
    if(i.bcSdPct>0)all.push(C('BC',Math.abs(s.vPerBcPct)*i.bcSdPct*k,Math.abs(s.hPerBcPct)*i.bcSdPct*k));
    if(i.groupMoa>0)all.push(C('Rifle & ammo group',g,g));
    if(i.aimSdMoa>0)all.push(C('Aim',aim,aim));
    var top=0;all.forEach(function(c){top=Math.max(top,c.total);});var across=Math.abs(Math.sin(wa));
    all.forEach(function(c){var small=c.total<0.5*top;
      if(c.name==='Wind direction'&&i.windMph<0.5)c.note='There is hardly any wind to call, so its direction can’t be off by much. Gusts from any side belong in Wind speed ±.';
      else if(c.name==='Wind direction'&&small&&across>0.94)c.note='Small in a full-value wind (3 or 9 o’clock): turned a little, it still blows almost fully across, so the drift hardly changes. It matters more in a quartering wind.';
      else if(c.name==='Wind speed'&&small&&across<0.34)c.note='Small in a head or tail wind: a wrong speed hardly moves the bullet sideways. Wind direction matters more here.';});
    return all.sort(function(a,b){return b.total-a.total;});
  },
  simulate:function(s,i,dia,seed,keep){
    var G0=rng(seed||1),k=s.rangeYd*HS.IN_PER_MIL_PER_YD,wa=i.windFromDeg*Math.PI/180,cc=i.windMph*Math.sin(wa),ch=i.windMph*Math.cos(wa);
    var gm=(i.groupMoa/HS.ES5)/MOA_PER_MIL,am=i.aimSdMoa/MOA_PER_MIL,hw=i.targetWidthIn/2+dia/2,hh=i.targetHeightIn/2+dia/2;
    var n=Math.min(200000,Math.max(100,i.shots|0)),hits=0,pts=[],flags=[],sx=0,sxx=0,sy=0,syy=0;
    for(var q=0;q<n;q++){
      var dv=G0()*i.mvSdFps,dbc=G0()*i.bcSdPct,dr=G0()*i.rangeSdYd,w=i.windMph+G0()*i.windSpeedSdMph,a=wa+G0()*i.windDirSdDeg*Math.PI/180;
      var dC=w*Math.sin(a)-cc,dH=w*Math.cos(a)-ch;
      var y=(s.vPerFps*dv+s.vPerBcPct*dbc+s.vPerYd*dr+s.vPerCrossMph*dC+s.vPerHeadMph*dH+G0()*gm+G0()*am)*k;
      var x=(s.hPerFps*dv+s.hPerBcPct*dbc+s.hPerYd*dr+s.hPerCrossMph*dC+s.hPerHeadMph*dH+G0()*gm+G0()*am)*k;
      var hit=i.round?((x/hw)*(x/hw)+(y/hh)*(y/hh)<=1):(Math.abs(x)<=hw&&Math.abs(y)<=hh);
      if(hit)hits++;if(q<keep){pts.push([x,y]);flags.push(hit);}
      sx+=x;sxx+=x*x;sy+=y;syy+=y*y;
    }
    function sdv(sum,sq){return Math.sqrt(Math.max(0,sq/n-(sum/n)*(sum/n)));}
    return {rangeYd:s.rangeYd,p:hits/n,hits:hits,shots:n,pts:pts,flags:flags,sdV:sdv(sy,syy),sdH:sdv(sx,sxx),causes:HitSim.causes(s,i)};
  },
  chartRanges:function(r){var top=Math.min(3000,Math.ceil(Math.max(r*1.5,r+300)/100)*100),step=top<=1000?50:(top<=2000?100:150),o=[];
    for(var x=100;x<=top+1e-9;x+=step)o.push(x);if(!o.some(function(v){return Math.abs(v-r)<1e-6;})&&r>HS.RANGE)o.push(r);return o.sort(function(a,b){return a-b;});},
  run:function(rifle,load,bullet,shot,i){
    var ranges=HitSim.chartRanges(i.rangeYd),sens=HitSim.sensitivities(rifle,load,bullet,shot,i.windMph,i.windFromDeg,ranges);
    var m=sens.filter(function(s){return Math.abs(s.rangeYd-i.rangeYd)<1e-6;})[0];
    var main=HitSim.simulate(m,i,fin(bullet.diameterIn,0),1,600);
    var chart=sens.map(function(s){return [s.rangeYd,HitSim.simulate(s,Object.assign({},i,{shots:4000}),fin(bullet.diameterIn,0),1,0).p];});
    return {main:main,chart:chart};
  }
};
function HitSimScreen(){
  var st=hsState(),me={},out={},timer=null,gen=0;
  function save(){lsSet('skyHitSim',JSON.stringify(st));clearTimeout(timer);timer=setTimeout(compute,250);}
  function ammoList(){var o=[];rifles().forEach(function(r){loadsOf(r.id).forEach(function(l){if(G().bullet(l.bulletId))o.push({r:r,l:l,title:r.name+' · '+l.name});});});return o;}
  function gearNow(){
    var all=ammoList();
    if(st.useSaved&&all.length){
      var pick=all.filter(function(a){return a.l.id===st.loadId;})[0]||all.filter(function(a){return a.l.id===activeId();})[0]||all[0];
      var p=G().flat(pick.l.id);p.loadId=pick.l.id;var g=G().engineGear(p);g.title=pick.title;g.pick=pick;return g;
    }
    return {rifle:{twistIn:st.twistIn,twistDirection:'RIGHT',sightHeightIn:st.sightHeightIn,sightOffsetIn:0},
      load:{muzzleVelocityFps:st.mvFps,zeroRangeYd:st.zeroYd,powderTempF:59,velocityPerDegFps:0,dragScaleFactor:1,zeroHeightIn:0,zeroOffsetIn:0},
      bullet:{id:'hs',name:'',diameterIn:st.diameterIn,weightGr:st.weightGr,lengthIn:null,dragModel:st.g1?'G1':'G7',bcPoints:[{bc:st.bc,minVelFps:0}],dragCurve:[],seasonalBcPoints:{}},
      title:Num.trim(st.weightGr,0)+' gr '+(st.g1?'G1':'G7')+' '+Num.fixed(st.bc,3)+' @ '+Units.text(Q.VELOCITY,st.mvFps,0)};
  }
  function ranged(g,lab,q,key,lo,hi,o){
    o=o||{};var s=S();var dec=o.decimals!==undefined?o.decimals:Units.decimals(q,s);
    var hint=o.sub||'';
    var nr=g.number(lab,q,st[key],function(v){
      var ok=v!==null&&v>=lo&&v<=hi;
      if(nr&&nr.label.sub){var u=Units.label(q,s);u=(!u||u==='°')?u:' '+u;
        nr.label.sub.textContent=(v===null||ok)?hint:'Use '+Num.trim(Units.toDisplay(q,lo,s),dec)+' to '+Num.trim(Units.toDisplay(q,hi,s),dec)+u;
        nr.label.sub.style.color=(v===null||ok)?'':'var(--error)';nr.label.sub.style.display=nr.label.sub.textContent?'':'none';}
      if(ok){st[key]=v;save();}
    },{decimals:o.decimals,sub:hint,unit:o.unit});
    if(nr.label.sub&&!hint)nr.label.sub.style.display='none';
    return nr;
  }
  me.render=function(col){
    toolbar({back:back,title:'Hit probability'});bottomNav(null);
    var f=new Form(col),s=S(),all=ammoList(),saved=st.useSaved&&all.length>0;
    f.header('Ammo');
    f.group(function(g){
      g.segmented('Use',['My ammo','Type it in'],saved?0:1,function(i){st.useSaved=i===0;save();rerender();});
      if(saved){
        var gg=gearNow(),idx=Math.max(0,all.indexOf(gg.pick));
        g.choice('Ammo',all.map(function(a){return a.title;}),idx,Units.text(Q.VELOCITY,gg.pick.l.muzzleVelocityFps,0)+' · zero '+dist(fin(gg.pick.l.zeroRangeYd,100)),
          function(i){st.loadId=all[i].l.id;save();rerender();});
      }else{
        if(st.useSaved)g.info('No saved ammo yet','','Type the ammo in below, or add ammo under My gear.');
        ranged(g,'Bullet diameter',Q.BULLET_LENGTH,'diameterIn',0.1,0.6);
        ranged(g,'Bullet weight',Q.WEIGHT,'weightGr',10,800);
        g.segmented('BC type',['G7','G1'],st.g1?1:0,function(i){st.g1=i===1;save();});
        ranged(g,'BC',Q.PLAIN,'bc',0.03,1.5,{decimals:3,unit:''});
        ranged(g,'Muzzle velocity',Q.VELOCITY,'mvFps',600,4800);
        ranged(g,'Zero distance',Q.DISTANCE,'zeroYd',10,1000);
        ranged(g,'Sight height',Q.LENGTH,'sightHeightIn',0,6);
        ranged(g,'Twist, 1 in',Q.BULLET_LENGTH,'twistIn',3,30,{decimals:1});
      }
    });
    f.header('Target');
    f.group(function(g){
      ranged(g,'Distance',Q.DISTANCE,'rangeYd',50,3000);
      ranged(g,'Width',Q.LENGTH,'targetWidthIn',0.25,400);
      ranged(g,'Height',Q.LENGTH,'targetHeightIn',0.25,400);
      g.segmented('Shape',['Rectangle','Round'],st.round?1:0,function(i){st.round=i===1;save();});
    });
    f.header('The wind you call');
    f.group(function(g){
      ranged(g,'Speed',Q.SPEED,'windMph',0,60);
      var clocks=[];for(var h=0;h<12;h++)clocks.push((h===0?12:h)+' o’clock'+({0:' (head wind)',3:' (from the right)',6:' (tail wind)',9:' (from the left)'}[h]||''));
      g.choice('From',clocks,Math.round(nrm(st.windFromDeg)/30)%12,null,function(h){st.windFromDeg=h*30;save();});
      var lw=liveWind();
      if(lw.ok){var b=el('button','sk-obtn','Use the Wind Lab meters ('+speedTxt(lw.speed)+', '+WindText.hour(lw.rel)+')');b.style.width='100%';
        on(b,'click',function(){st.windMph=+lw.speed.toFixed(1);st.windFromDeg=Math.round(lw.rel/30)*30%360;
          var a=root.weightedAvgWind?root.weightedAvgWind():null;if(a&&a.sd>0)st.windSpeedSdMph=+Math.min(30,a.sd).toFixed(1);save();rerender();});
        var wrap=el('div');wrap.style.width='100%';wrap.appendChild(b);g.custom(wrap);}
    });
    f.header('How far off you might be');
    f.note('Each is one standard deviation: about two shots in three are within it.');
    f.group(function(g){
      ranged(g,'Velocity spread (SD)',Q.VELOCITY,'mvSdFps',0,100);
      ranged(g,'BC, ± %',Q.PLAIN,'bcSdPct',0,20,{decimals:1,unit:''});
      ranged(g,'Range, ±',Q.DISTANCE,'rangeSdYd',0,200);
      ranged(g,'Wind speed, ±',Q.SPEED,'windSpeedSdMph',0,30);
      ranged(g,'Wind direction, ±',Q.ANGLE,'windDirSdDeg',0,90,{decimals:0,sub:'Matters most in a quartering wind; little in a full-value 3 or 9 o’clock wind'});
      ranged(g,'Group size, MOA',Q.PLAIN,'groupMoa',0,10,{decimals:2,unit:'',sub:'Your rifle and ammo’s typical 5-shot group'});
      ranged(g,'Aim, ± MOA',Q.PLAIN,'aimSdMoa',0,10,{decimals:2,unit:'',sub:'How steady you hold, from your position'});
    });
    f.header('Result');
    f.group(function(g){
      var c=el('div');c.style.width='100%';c.style.textAlign='center';c.style.padding='4px 0 8px';
      out.big=el('div','','—');out.big.style.cssText='font-size:44px;font-weight:700;color:var(--on)';
      out.sub=el('div','sk-note','');out.sub.style.padding='0';out.sub.style.textAlign='center';
      out.plot=el('div');out.plot.style.marginTop='8px';add(c,out.big,out.sub,out.plot);g.custom(c);
    });
    f.header('What spreads the shots');
    f.group(function(g){out.causes=el('div');out.causes.style.width='100%';g.custom(out.causes);});
    f.header('Hit chance by distance');
    f.group(function(g){out.chart=el('div');out.chart.style.width='100%';g.custom(out.chart);});
    var a=G().atmoOf(params());
    f.note('Every simulated shot is dialled for the distance and wind you call, then lands off centre by a random draw of the errors above. A shot counts as a hit when the bullet touches the target. Weather comes from the Calculator: '+
      Units.text(Q.TEMPERATURE,a.temperatureF,0)+', '+Units.text(Q.PRESSURE,a.pressureInHg)+', '+Units.text(Q.ALTITUDE,a.altitudeFt,0)+'.');
    compute();
  };
  function len(v,d){var s=S();return Num.trim(Units.toDisplay(Q.LENGTH,v,s),d===undefined?1:d)+' '+Units.label(Q.LENGTH,s);}
  function compute(){
    if(!out.big||!doc.body.contains(out.big))return;var g0=++gen;
    var g;try{g=gearNow();}catch(e){out.big.textContent='—';out.sub.textContent='Can’t work this out: '+(e.message||'check the ammo');return;}
    out.sub.textContent='Working it out…';
    setTimeout(function(){
      if(g0!==gen)return;
      var rep;try{var sh=shotOf(params());rep=HitSim.run(g.rifle,g.load,g.bullet,sh,st);}catch(e){out.big.textContent='—';out.sub.textContent='Can’t work this out: '+(e.message||'check the ammo');return;}
      show(rep,g);
    },10);
  }
  function show(rep,g){
    var r=rep.main,s=S();
    out.big.textContent=Math.round(r.p*100)+'%';out.big.setAttribute('aria-label','Hit chance '+Math.round(r.p*100)+' percent');
    out.sub.textContent='hit chance at '+dist(st.rangeYd)+' · '+r.hits.toLocaleString('en-US')+' of '+r.shots.toLocaleString('en-US')+' simulated shots\n'+
      len(st.targetWidthIn)+' × '+len(st.targetHeightIn)+' '+(st.round?'round target':'target')+' · '+g.title;
    out.plot.innerHTML='';out.plot.appendChild(targetPlot(st,r));
    out.causes.innerHTML='';var top=(r.causes[0]&&r.causes[0].total>0)?r.causes[0].total:1;
    r.causes.forEach(function(c){
      var row=el('div');row.style.padding='6px 4px';
      var line=el('div');line.style.display='flex';var nm=el('div','',c.name);nm.style.cssText='flex:1;font-size:16px;color:var(--on)';
      var tiny=Units.toDisplay(Q.LENGTH,c.total,s)<0.05,parts=tiny?['under 0.1 '+Units.label(Q.LENGTH,s)]:[];
      if(!tiny){if(c.vIn>=0.05*c.total)parts.push('↕ '+len(c.vIn));if(c.hIn>=0.05*c.total)parts.push('↔ '+len(c.hIn));}
      var vv=el('div','',parts.join('  '));vv.style.cssText='font-size:15px;color:var(--on-var)';add(line,nm,vv);row.appendChild(line);
      var bar=el('div');bar.style.cssText='height:8px;border-radius:4px;background:var(--s-highest);margin-top:4px;overflow:hidden';
      var fill=el('div');fill.style.cssText='height:100%;background:var(--primary);border-radius:4px;width:'+Math.max(1,Math.round(c.total/top*100))+'%';bar.appendChild(fill);row.appendChild(bar);
      if(c.note){var nt=el('div','',c.note);nt.style.cssText='font-size:15px;color:var(--on-var);padding-top:4px';row.appendChild(nt);}
      out.causes.appendChild(row);
    });
    var ex=el('div','','Typical miss from each cause at the target (about one standard deviation), up and down ↕ and side to side ↔. The biggest one is where to improve.');
    ex.style.cssText='font-size:15px;color:var(--on-var);padding:6px 4px 0';out.causes.appendChild(ex);
    out.chart.innerHTML='';out.chart.appendChild(hitChart(rep.chart.map(function(p){return [Units.toDisplay(Q.DISTANCE,p[0],s),p[1]];}),Units.toDisplay(Q.DISTANCE,st.rangeYd,s),Units.label(Q.DISTANCE,s)));
  }
  return me;
}
function targetPlot(i,r){
  var NS='http://www.w3.org/2000/svg',W=300,H=300,svg=doc.createElementNS(NS,'svg');svg.setAttribute('viewBox','0 0 '+W+' '+H);svg.style.cssText='width:100%;max-width:340px;height:auto;display:block;margin:0 auto';
  function mk(t,a){var e=doc.createElementNS(NS,t);for(var k in a)e.setAttribute(k,a[k]);svg.appendChild(e);return e;}
  var cx=W/2,cy=H/2,half=Math.max(Math.max(i.targetWidthIn,i.targetHeightIn)/2*1.15,2.6*Math.max(r.sdH,r.sdV)),k=(W/2-4)/half;
  var w=i.targetWidthIn/2*k,h=i.targetHeightIn/2*k;
  var shape=i.round?{t:'ellipse',a:{cx:cx,cy:cy,rx:w,ry:h}}:{t:'rect',a:{x:cx-w,y:cy-h,width:2*w,height:2*h}};
  mk(shape.t,Object.assign({fill:'var(--s-highest)',stroke:'var(--outline)','stroke-width':2},shape.a));
  mk('line',{x1:0,y1:cy,x2:W,y2:cy,stroke:'var(--outline-var)'});mk('line',{x1:cx,y1:0,x2:cx,y2:H,stroke:'var(--outline-var)'});
  r.pts.forEach(function(p,j){mk('circle',{cx:(cx+p[0]*k).toFixed(1),cy:(cy-p[1]*k).toFixed(1),r:2.5,fill:r.flags[j]?'var(--primary)':'var(--error)','fill-opacity':0.75});});
  svg.setAttribute('role','img');svg.setAttribute('aria-label','Target with '+r.pts.length+' simulated impacts, '+r.flags.filter(Boolean).length+' hits');
  return svg;
}
function hitChart(pts,mark,unit){
  var NS='http://www.w3.org/2000/svg',W=340,H=220,svg=doc.createElementNS(NS,'svg');svg.setAttribute('viewBox','0 0 '+W+' '+H);svg.style.cssText='width:100%;height:auto;display:block';
  function mk(t,a,txt){var e=doc.createElementNS(NS,t);for(var k in a)e.setAttribute(k,a[k]);if(txt!==undefined)e.textContent=txt;svg.appendChild(e);return e;}
  if(pts.length<2)return svg;
  var L=44,B=H-24,T=14,R=W-12,x0=pts[0][0],x1=pts[pts.length-1][0];
  function x(v){return L+(v-x0)/(x1-x0)*(R-L);}function y(p){return B-p*(B-T);}
  [0,0.5,1].forEach(function(p){mk('line',{x1:L,y1:y(p),x2:R,y2:y(p),stroke:'var(--outline-var)'});mk('text',{x:L-6,y:y(p)+4,'text-anchor':'end','font-size':12,fill:'var(--on-var)'},Math.round(p*100)+'%');});
  [[x0,'start',''],[(x0+x1)/2,'middle',''],[x1,'end',' '+unit]].forEach(function(t){mk('text',{x:x(t[0]),y:H-4,'text-anchor':t[1],'font-size':12,fill:'var(--on-var)'},Math.round(t[0])+t[2]);});
  mk('path',{d:pts.map(function(p,i){return (i?'L':'M')+x(p[0]).toFixed(1)+','+y(p[1]).toFixed(1);}).join(''),fill:'none',stroke:'var(--primary)','stroke-width':3,'stroke-linejoin':'round'});
  var m=pts.reduce(function(a,b){return Math.abs(b[0]-mark)<Math.abs(a[0]-mark)?b:a;});
  mk('circle',{cx:x(m[0]),cy:y(m[1]),r:6,fill:'var(--primary)'});
  var ly=y(m[1])-12<T?y(m[1])+22:y(m[1])-12;
  mk('text',{x:Math.min(R-14,Math.max(L+14,x(m[0]))),y:ly,'text-anchor':'middle','font-size':13,'font-weight':700,fill:'var(--on)'},Math.round(m[1]*100)+'%');
  return svg;
}

// ============================================================================
// MORE (Skyward MoreActivity) and SETTINGS (Skyward SettingsActivity)
// ============================================================================
function MoreScreen(){
  var me={tab:'more',onGear:true};
  me.render=function(col){
    toolbar({back:toWindLab,backLabel:'Back to Wind Lab',title:'More'});
    bottomNav(TABS,'more',goTab);
    var f=new Form(col),L=G().lib(),libn=L.bullets.filter(function(b){return b.source==='library';}).length,m=G().meta()||{};
    f.add(el('div','sk-gap'));
    f.group(function(g){
      g.nav('Import gear','','A Skyward Ballistics backup or bullet library (skyward-library JSON), or a Coriolis profiles file. Bullet library '+(libn?'('+libn+' bullets)':'not imported'),importGear);
      g.nav('Export gear','','Save all your firearms, ammo, bullets and target cards as a skyward-library file the Skyward app can import',function(){
        download('coriolis-gear-'+new Date().toISOString().slice(0,10)+'.json','application/json',G().exportText('master'));});
      g.nav('Sync with base station','',(m.status?m.status.charAt(0).toUpperCase()+m.status.slice(1):'Not synced yet')+(m.err?'\n⚠ '+m.err:''),function(){
        busy(true);Promise.resolve(G().sync()).then(function(){busy(false);rerender();var mm=G().meta()||{};snack(mm.err?'Not synced: '+mm.err:'Synced with the base station');});});
      g.nav('Settings','','Colours, units, how the answer is shown',function(){push(SettingsScreen());});
    });
    f.header('Wind Lab');
    f.group(function(g){
      g.nav('Back to Wind Lab','','Meters, forecast, log and sandbox',toWindLab);
      g.nav('Classic Coriolis solver','','The original one-page solver form. Same gear and settings',function(){closeSky();if(root.openClassicSolver)root.openClassicSolver();});
    });
    f.note('Skyward Ballistics in Coriolis Wind Lab. Your gear stays on this phone and on the base station; nothing goes to any other server.');
  };
  function importGear(){
    var inp=el('input');inp.type='file';inp.accept='.json,application/json,text/plain';inp.style.display='none';doc.body.appendChild(inp);
    on(inp,'change',function(){var file=inp.files&&inp.files[0];inp.remove();if(!file)return;
      var rd=new FileReader();rd.onload=function(){
        try{var r=G().importText(String(rd.result));refreshActive();rerender();
          alertBox('Imported',(r.rifles?r.rifles+' firearm'+(r.rifles===1?'':'s')+', ':'')+r.loads+' ammo, '+r.bullets+' bullet'+(r.bullets===1?'':'s')+' read'+(r.changed?'; '+r.changed+' added or updated.':'; nothing new.'));}
        catch(e){alertBox('Can’t import',e.message||String(e));}};
      rd.readAsText(file);});
    inp.click();
  }
  return me;
}
function SettingsScreen(){
  var me={};
  me.render=function(col){
    toolbar({back:back,title:'Settings'});bottomNav(null);
    var s=S(),f=new Form(col);
    function put(k,v,re){var n=S();n[k]=v;saveS(n);if(re)rerender();}
    f.header('Display');
    f.group(function(g){
      g.choice('Motion',['Full','Reduced'],s.motion==='reduced'?1:0,'Reduced: the logo stays still and changes happen at once. The phone’s own "Remove animations" setting does the same',function(i){put('motion',i===1?'reduced':'full');});
      var th=['system','light','dark'];
      g.choice('Colours',['Same as phone','Light (daylight)','Dark (low light)'],Math.max(0,th.indexOf(s.theme)),null,function(i){put('theme',th[i]);});
    });
    f.header('Units');
    f.group(function(g){
      g.choice('Distance',['Yards','Meters'],s.distanceUnit==='m'?1:0,null,function(i){
        var u=i===1?'m':'yd',o=S();if(o.distanceUnit===u)return;var n=JSON.parse(JSON.stringify(o));n.distanceUnit=u;
        function keep(yd){return Units.fromDisplay(Q.DISTANCE,Math.round(Units.toDisplay(Q.DISTANCE,yd,o)),n);}
        n.solutionStepYd=keep(o.solutionStepYd);n.tableStepYd=keep(o.tableStepYd);n.tableMaxYd=keep(o.tableMaxYd);saveS(n);rerender();});
      g.choice('Weather',['Imperial (ft, inHg, °F)','Metric (m, hPa, °C)'],s.atmosphereUnits==='metric'?1:0,null,function(i){put('atmosphereUnits',i===1?'metric':'imperial',true);});
      g.choice('Everything else',['Imperial (in, fps, ft-lb, mph)','Metric (cm, m/s, J, m/s)'],s.otherUnits==='metric'?1:0,'Bullet path, speed, energy and wind',function(i){put('otherUnits',i===1?'metric':'imperial',true);});
    });
    f.note('These units are for this solver. The Wind Lab meters keep their own.');
    f.header('Entering a shot');
    f.group(function(g){
      g.sw('Enter density altitude',s.useDensityAltitude,'Type one density-altitude number and the temperature, instead of altitude, pressure and humidity',function(c){put('useDensityAltitude',c);});
      g.number('Compass declination',Q.ANGLE,fin(ls('compassDecl','0'),0),function(v){if(v!==null&&Math.abs(v)<=60)lsSet('compassDecl',v);},
        {signed:true,decimals:1,sub:'Added to the phone compass reading. + east, − west. 0 keeps magnetic north'});
      g.number('Distance step',Q.DISTANCE,s.solutionStepYd,function(v){if(v!==null&&v>0)put('solutionStepYd',v);},{decimals:1,sub:'How far each tap of − or + moves the distance'});
    });
    f.header('Showing the answer');
    f.group(function(g){
      g.sw('Show + and − instead of letters',s.usePlusMinus,'Off: U / D for up and down, R / L for right and left. On: + and −',function(c){put('usePlusMinus',c);});
      var pr=['tenth','hundredth','click'];
      g.choice('Round the answer to',['One decimal (0.1)','Two decimals (0.01)','The nearest click'],Math.max(0,pr.indexOf(s.outputPrecision)),null,function(i){put('outputPrecision',pr[i]);});
    });
    f.header('Trajectory table');
    f.group(function(g){
      g.number('Row spacing',Q.DISTANCE,s.tableStepYd,function(v){if(v!==null&&v>=1)put('tableStepYd',v);},{decimals:1,sub:'Distance between rows of the table'});
      g.number('Furthest distance',Q.DISTANCE,s.tableMaxYd,function(v){if(v!==null&&v>=50&&v<=5000)put('tableMaxYd',v);},{sub:'Where the table stops'});
      g.sw('Furthest distance first',s.tableSortDescending,'Lists the table from far to near',function(c){put('tableSortDescending',c);});
    });
  };
  return me;
}

// ============================================================================
// Public API: Wind Lab opens this when the Solver tab is tapped.
// ============================================================================
var gearHooked=false;
function openSky(where){
  if(!root.Gear||!root.SkyEngine)return false;
  build();
  if(!gearHooked){gearHooked=true;
    // Edits made here are this phone's own: Wind Lab must not call them "updated on another device".
    ['saveRecord','removeRecord','importText'].forEach(function(k){var f=G()[k];if(typeof f!=='function')return;
      G()[k]=function(){var q=root.__skyQuiet;root.__skyQuiet=true;try{return f.apply(this,arguments);}finally{root.__skyQuiet=q;}};});
G().onChange(function(){if(!openFlag)return;var sc=current();if(sc&&sc.onGear&&!doc.querySelector('#sky input:focus,#sky textarea:focus'))rerender();});}
  var t=where||ls('skyTab','calc');if(['calc','gear','analysis','more'].indexOf(t)<0)t='calc';tab=t;
  stack=[screenFor(t)];openFlag=true;skipPop=0;
  try{root.history.pushState({sky:1},'');}catch(e){}
  shell.root.className='';applyTheme();
  doc.documentElement.style.overflow='hidden';
  show(0);
  if(!still())splash();
  try{G().sync();}catch(e){}
  return true;
}
function closeSky(fromPop){
  if(!openFlag)return;openFlag=false;
  dismissOverlays();
  var n=stack.length;stack=[];
  shell.root.className='hide';shell.main.innerHTML='';shell.res.innerHTML='';
  doc.documentElement.style.overflow='';
  if(!fromPop){try{root.history.go(-n);}catch(e){}}
  if(root.render)try{root.render();}catch(e){}
}
root.SkyUI={
  open:openSky,
  close:function(){closeSky(false);},
  isOpen:function(){return openFlag;},
  onData:function(){if(!openFlag)return;var sc=current();if(sc&&sc.onData)try{sc.onData();}catch(e){}},
  logo:LOGO,
  screen:function(){var sc=current();return sc?(sc.tab||sc.sub||''):'';},
  // for tests
  _solveNow:function(){return solveNow();},_params:function(){return params();},_goTab:function(id){goTab(id);}
};
})(typeof window!=='undefined'?window:this);

)SKYJS";
