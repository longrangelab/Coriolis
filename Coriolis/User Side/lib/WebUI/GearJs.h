#ifndef GEAR_JS_H
#define GEAR_JS_H

/**
 * GearJs.h  --  rifles / loads / bullets on the phone   (served as /gear.js)
 * ==========================================================================
 * Gear is kept in Skyward Ballistics' own library format ("skyward-library",
 * version 1: rifles, loads, bullets, deleted, cards, shots), so a file exported
 * from Skyward imports here unchanged and a Coriolis export imports into Skyward.
 *
 * Where it lives:
 *   * master copy on the base station's flash (GET/POST /gear, see WebUI.h),
 *     shared by every phone that joins the access point;
 *   * a cache in this phone's localStorage ('gearLib'), so the page works and
 *     edits are kept even when the base station can't be reached.
 * Merging is Skyward's rule: per record id the newest updatedAt wins, and a
 * deletion (tombstone) beats any copy saved before it.
 *
 * The solver screen still edits one flat "profile" (the LOAD fields in
 * WebUI.h). Gear.flat() / Gear.saveFlat() translate between that flat profile
 * and the rifle + load + bullet records. Group offsets are stored on the load
 * as Skyward's zeroHeightIn / zeroOffsetIn (inches at the zero range) and shown
 * in MOA as before.
 *
 * Old Coriolis profiles ('profiles' in localStorage, rifle_profiles.json) are
 * converted on first run and can still be imported; the old key is left in
 * place as a backup.
 */

static const char GEAR_JS[] PROGMEM = R"GEARJS(
(function(root){
'use strict';
var KEY='gearLib',META='gearMeta',FORMAT='skyward-library';
var TOMB_DAYS=180,IN_PER_MOA_100=1.04720;   // 1 MOA = 1.0472 in at 100 yd
function now(){return Date.now();}
function ls(){try{return root.localStorage;}catch(e){return null;}}
function lsGet(k){try{var s=ls();return s?s.getItem(k):null;}catch(e){return null;}}
function lsSet(k,v){try{var s=ls();if(s)s.setItem(k,v);return true;}catch(e){return false;}}
function uuid(){
  if(root.crypto&&root.crypto.randomUUID)return root.crypto.randomUUID();
  var h='0123456789abcdef',s='';
  for(var i=0;i<36;i++){if(i===8||i===13||i===18||i===23){s+='-';continue;}
    var r=Math.floor(Math.random()*16);s+=h[i===14?4:(i===19?(r&3)|8:r)];}
  return s;
}
// Stable id from a name, so re-importing the same old Coriolis file updates
// instead of duplicating.
function stableId(key){
  function fnv(s,seed){var h=seed>>>0;for(var i=0;i<s.length;i++){h^=s.charCodeAt(i);h=Math.imul(h,16777619)>>>0;}return ('0000000'+h.toString(16)).slice(-8);}
  var a=fnv(key,2166136261),b=fnv(key,0x9747b28c),c=fnv(key,0x1b873593),d=fnv(key,0xcc9e2d51);
  return a+'-'+b.slice(0,4)+'-5'+b.slice(5,8)+'-8'+c.slice(1,4)+'-'+c.slice(4)+d;
}
function n(v,d){v=parseFloat(v);return isFinite(v)?v:d;}
function r(v,dp){var m=Math.pow(10,dp);return Math.round(v*m)/m;}
function clone(o){return JSON.parse(JSON.stringify(o));}
function emptyLib(){return {rifles:[],loads:[],bullets:[],deleted:[],cards:[],shots:[]};}
function hash(s){   // cyrb53: 53-bit content hash
  var h1=0xdeadbeef,h2=0x41c6ce57;
  for(var i=0;i<s.length;i++){var c=s.charCodeAt(i);h1=Math.imul(h1^c,2654435761);h2=Math.imul(h2^c,1597334677);}
  h1=Math.imul(h1^(h1>>>16),2246822507)^Math.imul(h2^(h2>>>13),3266489909);
  h2=Math.imul(h2^(h2>>>16),2246822507)^Math.imul(h1^(h1>>>13),3266489909);
  return (h2>>>0).toString(16)+':'+(h1>>>0).toString(16)+':'+s.length;
}
function canon(lib){   // order-independent content key, for "did anything change"
  function srt(a){return (a||[]).slice().sort(function(x,y){return x.id<y.id?-1:x.id>y.id?1:0;});}
  return JSON.stringify([srt(lib.rifles),srt(lib.loads),srt(lib.bullets),srt(lib.cards),srt(lib.shots),
    (lib.deleted||[]).map(function(t){return t.kind+'|'+t.id+'|'+t.deletedAt;}).sort()]);
}

var lib=emptyLib(),meta={serverVer:-1,lastSync:0,status:'',err:''};
var listeners=[];
function changed(){listeners.forEach(function(f){try{f();}catch(e){}});}
function persist(){
  lsSet(META,JSON.stringify(meta));
  if(!lsSet(KEY,JSON.stringify(lib)))meta.err='phone storage full: gear kept on the base station only';
}
function load(){
  try{var t=JSON.parse(lsGet(KEY));if(t&&t.rifles)lib=normalize(t);}catch(e){}
  try{var m=JSON.parse(lsGet(META));if(m)meta=Object.assign(meta,m);}catch(e){}
  delete meta.serverCanon;   // older builds kept a whole second copy here
}

// ---------- merge (Skyward rule) ----------
var KINDS=[['rifles','rifle'],['loads','load'],['bullets','bullet'],['cards','card'],['shots','shot']];
function tombKind(listKey,rec){return listKey==='bullets'&&rec&&rec.source==='library'?'library':(KINDS.filter(function(k){return k[0]===listKey;})[0][1]);}
// Merge `inc` into `base` in place. manual=true for a file the user picked:
// records without a timestamp count as "now", so the import wins.
// Later edit wins; two different copies saved in the same millisecond are settled
// the same way on every phone (by content), so they can't flip-flop forever.
function newer(x,y){
  var a=+x.updatedAt||0,b=+y.updatedAt||0;if(a!==b)return a>b;
  var sx=JSON.stringify(x),sy=JSON.stringify(y);return sx!==sy&&sx>sy;
}
function mergeInto(base,inc,manual){
  var t=now(),tombs={},changedN=0;
  (base.deleted||[]).concat(inc.deleted||[]).forEach(function(d){
    var k=d.kind+'|'+d.id;if(!tombs[k]||tombs[k].deletedAt<d.deletedAt)tombs[k]={kind:d.kind,id:d.id,deletedAt:+d.deletedAt||0};
  });
  KINDS.forEach(function(k){
    var key=k[0],have={};
    base[key]=base[key]||[];
    base[key].forEach(function(x,i){have[x.id]=i;});
    (inc[key]||[]).forEach(function(x0){
      if(!x0||!x0.id)return;
      var x=clone(x0);
      if(!(x.updatedAt>0))x.updatedAt=manual?t:0;
      var tb=tombs[tombKind(key,x)+'|'+x.id];
      var i=have[x.id];
      if(i===undefined){
        if(tb&&tb.deletedAt>=x.updatedAt)return;      // deleted after that copy was saved
        have[x.id]=base[key].length;base[key].push(x);changedN++;
      }else if(newer(x,base[key][i])){
        base[key][i]=x;changedN++;
      }
    });
    var before=base[key].length;
    base[key]=base[key].filter(function(x){var tb=tombs[tombKind(key,x)+'|'+x.id];return !(tb&&tb.deletedAt>=(x.updatedAt||0));});
    changedN+=before-base[key].length;
  });
  var cutoff=t-TOMB_DAYS*864e5;
  base.deleted=Object.keys(tombs).map(function(k){return tombs[k];}).filter(function(d){return d.deletedAt>=cutoff;});
  return changedN;
}

// ---------- file formats ----------
function normalize(o){
  var l=emptyLib();
  // keep only real records: a bad file must not reach the base station and break every phone
  function okId(x){return x&&typeof x==='object'&&!Array.isArray(x)&&(typeof x.id==='string'&&x.id!==''||typeof x.id==='number');}
  ['rifles','loads','bullets','cards','shots'].forEach(function(k){if(Array.isArray(o[k]))l[k]=o[k].filter(okId);});
  ['rifles','loads','bullets'].forEach(function(k){l[k].forEach(function(x){if(typeof x.name!=='string')x.name=x.name==null||x.name===''?'Unnamed':String(x.name);});});
  if(Array.isArray(o.deleted))l.deleted=o.deleted.filter(function(t){return okId(t)&&typeof t.kind==='string';});
  return l;
}
// Old Coriolis profile -> rifle + load + bullet (stable ids from the name)
function fromLegacy(name,p){
  var rid=stableId('coriolis-rifle:'+name),lid=stableId('coriolis-load:'+name),bid=stableId('coriolis-bullet:'+name);
  var zero=n(p.zero,100),inPerMoa=IN_PER_MOA_100*zero/100;
  var eM=n(p.zeroElevMag,0)*(p.zeroElevDir==='low'?-1:1),wM=n(p.zeroWindMag,0)*(p.zeroWindDir==='left'?-1:1);
  var model=(''+p.model)==='1'?'G1':'G7',bc=n(p.bc,0.243),wgt=n(p.wgt,175),cal=n(p.cal,0.308);
  var t=0;   // no timestamp: an explicit re-import or a newer edit elsewhere decides
  return {
    rifle:{id:rid,name:name,twistIn:n(p.twist,10),twistDirection:(''+p.twistDir)==='-1'?'LEFT':'RIGHT',
           sightHeightIn:n(p.sh,1.75),sightOffsetIn:0,updatedAt:t},
    bullet:{id:bid,maker:'',name:wgt+' gr '+model+' '+bc,diameterIn:cal,weightGr:wgt,
            lengthIn:p.blen!==undefined&&p.blen!==''?n(p.blen,null):null,dragModel:model,
            bcPoints:[{bc:bc,minVelFps:0}],dragCurve:[],source:'user',updatedAt:t},
    load:{id:lid,rifleId:rid,name:name,bulletId:bid,muzzleVelocityFps:n(p.mv,2700),
          zeroRangeYd:zero,zeroHeightIn:r(eM*inPerMoa,4),zeroOffsetIn:r(wM*inPerMoa,4),
          powderTempF:59,velocityPerDegFps:0,dragScaleFactor:1,updatedAt:t}
  };
}
// Parse any supported file: Skyward library (any kind, incl. the older
// dragcurve/arcledger names) or an old Coriolis rifle_profiles.json.
function parseFile(text){
  var o;
  try{o=JSON.parse(String(text==null?'':text).replace(/^\uFEFF/,''));}
  catch(e){var err=new Error(String(text||'').trim()?'This file is not valid JSON. It may be cut off, or not a gear file.':'The file is empty.');err.unreadable=true;throw err;}
  if(o&&typeof o==='object'&&typeof o.format==='string'){
    if([FORMAT,'dragcurve-library','arcledger-library'].indexOf(o.format)<0)
      throw new Error(o.format==='epv5-user-data'?'This is an Empirical Precision backup. Import it into Skyward Ballistics first, then export from there.':
        'Not a Skyward Ballistics library (format "'+o.format+'").');
    if(o.version!==1)throw new Error('Skyward library version '+o.version+' is not one this page reads (expected 1).');
    var l=normalize(o);
    if((o.kind||'').toLowerCase()!=='master'){l.deleted=[];l.cards=[];l.shots=[];}
    // A load whose rifle isn't in the file: Skyward exports carry rifleName.
    l.loads.forEach(function(ld){if(ld.rifleName&&!l.rifles.some(function(x){return x.id===ld.rifleId;})&&!lib.rifles.some(function(x){return x.id===ld.rifleId;})){
      var match=lib.rifles.filter(function(x){return x.name===ld.rifleName;})[0];
      if(match)ld.rifleId=match.id;else l.rifles.push({id:ld.rifleId,name:ld.rifleName,twistIn:8,twistDirection:'RIGHT',sightHeightIn:1.5,updatedAt:0});
    }delete ld.rifleName;});
    return {lib:l,kind:(o.kind||'').toLowerCase(),app:o.app||''};
  }
  if(o&&typeof o==='object'&&!Array.isArray(o)){
    var l2=emptyLib(),c=0;
    Object.keys(o).forEach(function(name){var p=o[name];if(!p||typeof p!=='object'||p.mv===undefined)return;
      var g=fromLegacy(name,p);l2.rifles.push(g.rifle);l2.bullets.push(g.bullet);l2.loads.push(g.load);c++;});
    if(c)return {lib:l2,kind:'coriolis-profiles',app:'Coriolis'};
  }
  throw new Error('Not a Skyward Ballistics library or a Coriolis profiles file.');
}
function exportText(kind){
  kind=kind||'master';
  var rn={};lib.rifles.forEach(function(x){rn[x.id]=x.name;});
  var o={format:FORMAT,version:1,kind:kind,exportedAt:new Date().toISOString().replace(/\.\d+Z$/,'Z'),app:'Coriolis',
    units:{length:'in',weight:'gr',velocity:'ft/s',distance:'yd',temperature:'degF',pressure:'inHg'},
    rifles:lib.rifles,loads:lib.loads.map(function(l){var c=clone(l);if(rn[l.rifleId])c.rifleName=rn[l.rifleId];return c;}),
    bullets:lib.bullets};
  if(kind==='master'){o.deleted=lib.deleted;o.cards=lib.cards;o.shots=lib.shots;}
  if(kind==='bullets'){o.rifles=[];o.loads=[];}
  return JSON.stringify(o,null,1);
}
function importText(text){
  var res=parseFile(text);
  var before=canon(lib);
  var c=mergeInto(lib,res.lib,true);
  persist();if(canon(lib)!==before){changed();schedulePush();}
  return {changed:c,loads:res.lib.loads.length,bullets:res.lib.bullets.length,rifles:res.lib.rifles.length,kind:res.kind};
}

// ---------- lookups ----------
function byId(list,id){for(var i=0;i<list.length;i++)if(list[i].id===id)return list[i];return null;}
function rifle(id){return byId(lib.rifles,id);}
function loadRec(id){return byId(lib.loads,id);}
function bullet(id){return byId(lib.bullets,id);}
function label(l){var rf=rifle(l.rifleId),nm=String(l.name==null?'':l.name);return (!rf||!rf.name||rf.name===nm)?nm:(String(rf.name)+' · '+nm);}
function loadsSorted(){return lib.loads.slice().sort(function(a,b){var x=label(a).toLowerCase(),y=label(b).toLowerCase();return x<y?-1:x>y?1:0;});}

// ---------- flat profile <-> records ----------
function bandsToText(rows){
  return rows.slice().sort(function(a,b){return b.minVelFps-a.minVelFps;}).map(function(p){return (+p.bc)+'@'+(+(p.minVelFps||0));}).join(', ');
}
function parseBands(t){
  if(t===undefined||t===null||(''+t).trim()==='')return null;
  var rows=[];
  (''+t).split(/[,;\n]+/).forEach(function(part){
    part=part.trim();if(!part)return;
    var m=part.match(/^([0-9.]+)\s*(?:@\s*([0-9.]+))?$/);
    if(!m)throw new Error('BC bands: write them like 0.315@2600, 0.305@2000, 0.295');
    rows.push({bc:parseFloat(m[1]),minVelFps:m[2]?parseFloat(m[2]):0});
  });
  if(!rows.length)return null;
  rows.forEach(function(x){if(!(x.bc>0))throw new Error('BC bands: every BC must be above 0');});
  return rows.sort(function(a,b){return b.minVelFps-a.minVelFps;});
}
// Custom drag curve as text: "mach:cd" pairs, e.g. "0.5:0.142, 0.9:0.150,
// 1.0:0.290". One pair per line also works ("0.5,0.142" or "0.5 0.142"), so a
// two-column table pasted from a spreadsheet or Skyward's CSV reads as is.
function curveToText(pts){
  return (pts||[]).slice().sort(function(a,b){return a.mach-b.mach;}).map(function(p){return (+p.mach)+':'+(+p.cd);}).join(', ');
}
function parseCurve(t){
  if(t===undefined||t===null||(''+t).trim()==='')return null;
  t=''+t;var pairs=[],lines=t.split(/\r?\n/).map(function(x){return x.trim();}).filter(Boolean);
  var items=(lines.length>1&&!/:/.test(t))?lines:t.split(/[,;\n]+/);
  items.forEach(function(it){
    it=it.trim();if(!it)return;
    var m=it.match(/^([0-9]*\.?[0-9]+)\s*[:,\t ]\s*([0-9]*\.?[0-9]+)$/);
    if(!m){if(/^[a-z]/i.test(it))return;   // a header row such as "mach,cd"
      throw new Error('Drag curve: write Mach:Cd pairs like 0.5:0.142, 0.9:0.150, 1.0:0.290');}
    pairs.push({mach:parseFloat(m[1]),cd:parseFloat(m[2])});
  });
  if(pairs.length<2)throw new Error('Drag curve: needs at least 2 Mach:Cd points');
  pairs.forEach(function(x){if(!(x.cd>0))throw new Error('Drag curve: every Cd must be above 0');});
  pairs.sort(function(a,b){return a.mach-b.mach;});
  for(var i=1;i<pairs.length;i++)if(pairs[i].mach===pairs[i-1].mach)throw new Error('Drag curve: Mach '+pairs[i].mach+' appears twice');
  return pairs;
}
function modelCode(m){return m==='G1'?'1':(m==='CUSTOM'?'C':'7');}
function modelName(c){c=''+c;return c==='1'?'G1':(c==='C'?'CUSTOM':'G7');}
// Flat solver profile for a load (what the LOAD fields in the page show).
function flat(loadId){
  var l=loadRec(loadId);if(!l)return null;
  var rf=rifle(l.rifleId)||{},b=bullet(l.bulletId)||{};
  var model=l.dragModelOverride||b.dragModel||'G7';
  var rows=(l.bcPointsOverride&&l.bcPointsOverride.length)?l.bcPointsOverride:(b.bcPoints||[]);
  var top=rows.slice().sort(function(a,b){return b.minVelFps-a.minVelFps;})[0];
  var zero=n(l.zeroRangeYd,100),inPerMoa=IN_PER_MOA_100*zero/100;
  var eM=n(l.zeroHeightIn,0)/inPerMoa,wM=n(l.zeroOffsetIn,0)/inPerMoa;
  return {
    mv:n(l.muzzleVelocityFps,2700),bc:top?top.bc:(model==='CUSTOM'?'':0.243),model:modelCode(model),
    bcBands:rows.length>1?bandsToText(rows):'',dragCurve:curveToText(b.dragCurve),
    wgt:n(b.weightGr,175),cal:n(b.diameterIn,0.308),twist:n(rf.twistIn,10),
    blen:(b.lengthIn>0)?b.lengthIn:'',twistDir:rf.twistDirection==='LEFT'?-1:1,
    sh:n(rf.sightHeightIn,1.75),zero:zero,
    zeroElevMag:r(Math.abs(eM),2),zeroElevDir:eM<0?'low':'high',
    zeroWindMag:r(Math.abs(wM),2),zeroWindDir:wM<0?'left':'right',
    powderTempF:n(l.powderTempF,59),mvPerF:n(l.velocityPerDegFps,0),dragScale:n(l.dragScaleFactor,1),
    bulletId:l.bulletId||''
  };
}
function same(a,b){return JSON.stringify(a)===JSON.stringify(b);}
function put(list,rec){var i=-1;list.forEach(function(x,k){if(x.id===rec.id)i=k;});if(i<0)list.push(rec);else list[i]=rec;}
// An edit must beat the copy it replaces even when that copy came from a
// device whose clock runs ahead, so its stamp is never older than the old one.
function stamp(list,rec,old){if(!old||!same(Object.assign({},old,{updatedAt:0}),Object.assign({},rec,{updatedAt:0}))){rec.updatedAt=Math.max(now(),(old?(+old.updatedAt||0):0)+1);put(list,rec);return true;}return false;}
function bulletFromFlat(p,base){
  var model=modelName(p.model);
  var rows=parseBands(p.bcBands)||(model==='CUSTOM'?(base&&base.bcPoints)||[]:[{bc:n(p.bc,0.243),minVelFps:0}]);
  var b=Object.assign({},base||{id:uuid(),maker:'',name:'',source:'user'});
  b.diameterIn=n(p.cal,0.308);b.weightGr=n(p.wgt,175);
  b.lengthIn=(p.blen!==''&&p.blen!==undefined&&n(p.blen,0)>0)?n(p.blen,0):null;
  // A typed curve replaces the bullet's; a blank field keeps whatever it has.
  b.dragModel=model;b.bcPoints=rows;b.dragCurve=parseCurve(p.dragCurve)||(base&&base.dragCurve)||[];
  if(model==='CUSTOM'&&b.dragCurve.length<2)throw new Error('This bullet has no drag curve. Pick G1 or G7.');
  // names this page made up ("175 gr G7 0.243") follow the data; real names stay
  if(!b.name||AUTO_NAME.test(b.name))b.name=b.weightGr+' gr '+model+(model==='CUSTOM'?'':' '+rows[0].bc);
  return b;
}
var AUTO_NAME=/^[0-9.]+ gr (G1|G7|CUSTOM)( [0-9.]+)?$/;
function bulletMatches(b,p){
  if(!b)return false;
  var t=bulletFromFlat(p,b);
  return same([t.diameterIn,t.weightGr,t.lengthIn,t.dragModel,t.bcPoints,t.dragCurve],[b.diameterIn,b.weightGr,b.lengthIn||null,b.dragModel,b.bcPoints,b.dragCurve||[]]);
}
// Save a flat profile as load `name`. loadId = the load it was opened from
// (null for a brand-new one); asNew = "Save As": a new load on the same rifle.
// Returns the load id. Throws on bad input.
// Does saving p change twist/sight height on a rifle other loads also use?
// Returns {name, others} so the page can ask, or null.
function rifleShare(p,loadId,asNew){
  var src=loadId?loadRec(loadId):null,rf0=src?rifle(src.rifleId):null;if(!rf0)return null;
  var dir=(''+p.twistDir)==='-1'?'LEFT':'RIGHT';
  if(n(p.twist,10)===rf0.twistIn&&dir===rf0.twistDirection&&n(p.sh,1.75)===rf0.sightHeightIn)return null;
  var others=lib.loads.filter(function(x){return x.rifleId===rf0.id&&(asNew||x.id!==loadId);}).length;
  return others?{name:rf0.name,others:others}:null;
}
function saveFlat(name,p,loadId,asNew,rifleAll){
  name=(''+name).trim();if(!name)throw new Error('name required');
  var src=loadId?loadRec(loadId):null,l0=asNew?null:src;
  var l=l0?clone(l0):(src?Object.assign(clone(src),{id:uuid()}):{id:uuid(),rifleId:'',bulletId:''});
  var rf0=src?rifle(src.rifleId):null;
  var rf=rf0?clone(rf0):{id:uuid(),name:name,sightOffsetIn:0};
  // Changing twist/sight height on a rifle other loads share: change it for
  // all of them only when asked (rifleAll); otherwise this load gets its own.
  if(rf0&&!rifleAll&&rifleShare(p,loadId,asNew)){rf0=null;rf={id:uuid(),name:name,sightOffsetIn:0};}
  // a rifle that was made for this one load follows its rename
  if(rf0&&l0&&rf0.name===l0.name&&l0.name!==name&&lib.loads.filter(function(x){return x.rifleId===rf0.id;}).length===1)rf.name=name;
  rf.twistIn=n(p.twist,10);rf.twistDirection=(''+p.twistDir)==='-1'?'LEFT':'RIGHT';rf.sightHeightIn=n(p.sh,1.75);
  // bullet: keep the stored one when nothing about it changed; otherwise edit
  // it, or a copy of it when it is a library bullet or another load uses it
  var b0=bullet(p.bulletId||l.bulletId)||null,b;
  if((l.bcPointsOverride&&l.bcPointsOverride.length)||l.dragModelOverride){
    // this load overrides its bullet's BC: the edit goes back to the override
    var tb=bulletFromFlat(p,b0);
    l.dragModelOverride=(b0&&tb.dragModel===b0.dragModel)?null:tb.dragModel;
    l.bcPointsOverride=tb.bcPoints;
    b=b0?clone(b0):tb;
    if(b0&&!same([b0.diameterIn,b0.weightGr,b0.lengthIn||null,b0.dragCurve||[]],[tb.diameterIn,tb.weightGr,tb.lengthIn,tb.dragCurve])){
      b=Object.assign(clone(b0),{id:uuid(),source:'user',name:(b0.name||'bullet')+' (edited)',diameterIn:tb.diameterIn,weightGr:tb.weightGr,lengthIn:tb.lengthIn,dragCurve:tb.dragCurve});
      delete b.libraryOrder;delete b.collection;
    }
  }else if(b0&&bulletMatches(b0,p)){b=clone(b0);}
  else{
    var shared=b0&&(b0.source==='library'||lib.loads.some(function(x){return x.bulletId===b0.id&&x.id!==l.id;}));
    b=bulletFromFlat(p,shared?null:b0);
    // a made-up name ("175 gr G7 0.243") is just regenerated for the copy
    if(shared){if(!AUTO_NAME.test(b0.name||''))b.name=(b0.name||'bullet')+' (edited)';b.maker=b0.maker||'';if(!parseCurve(p.dragCurve))b.dragCurve=b0.dragCurve||[];}
  }
  var zero=n(p.zero,100),inPerMoa=IN_PER_MOA_100*zero/100;
  var eM=n(p.zeroElevMag,0)*(p.zeroElevDir==='low'?-1:1),wM=n(p.zeroWindMag,0)*(p.zeroWindDir==='left'?-1:1);
  l.name=name;l.rifleId=rf.id;l.bulletId=b.id;l.muzzleVelocityFps=n(p.mv,2700);l.zeroRangeYd=zero;
  // keep the stored inches when the MOA shown (2 decimals) did not change
  var oe=l0?n(l0.zeroHeightIn,0):NaN,ow=l0?n(l0.zeroOffsetIn,0):NaN,ozr=l0?n(l0.zeroRangeYd,100):NaN;
  var same2=function(a,b){return Math.abs(a-b)<0.005;};
  l.zeroHeightIn=(ozr===zero&&same2(oe/inPerMoa,eM))?oe:r(eM*inPerMoa,4);
  l.zeroOffsetIn=(ozr===zero&&same2(ow/inPerMoa,wM))?ow:r(wM*inPerMoa,4);
  // optional Skyward fields: leave them out while they hold the default, so
  // saving an untouched load does not count as an edit
  function opt(k,v,d){if(l[k]!==undefined&&l[k]!==null||v!==d)l[k]=v;}
  opt('powderTempF',n(p.powderTempF,59),59);opt('velocityPerDegFps',n(p.mvPerF,0),0);opt('dragScaleFactor',n(p.dragScale,1),1);
  var any=false;
  any=stamp(lib.rifles,rf,rf0)||any;
  any=stamp(lib.bullets,b,bullet(b.id))||any;
  any=stamp(lib.loads,l,l0)||any;
  persist();if(any){changed();schedulePush();}
  return l.id;
}
function tomb(kind,id){
  var list={rifle:lib.rifles,load:lib.loads,bullet:lib.bullets,library:lib.bullets}[kind]||[],t=0;
  list.forEach(function(x){if(x.id===id)t=+x.updatedAt||0;});
  lib.deleted.push({kind:kind,id:id,deletedAt:Math.max(now(),t+1)});
}
function removeLoad(loadId){
  var l=loadRec(loadId);if(!l)return;
  tomb('load',loadId);lib.loads=lib.loads.filter(function(x){return x.id!==loadId;});
  // the rifle goes too when this was its only load and it was made here for it
  var rf=rifle(l.rifleId);
  if(rf&&!lib.loads.some(function(x){return x.rifleId===rf.id;})&&rf.name===l.name){tomb('rifle',rf.id);lib.rifles=lib.rifles.filter(function(x){return x.id!==rf.id;});}
  var b=bullet(l.bulletId);
  if(b&&b.source!=='library'&&!lib.loads.some(function(x){return x.bulletId===b.id;})&&/^\d/.test(b.name||'')){tomb('bullet',b.id);lib.bullets=lib.bullets.filter(function(x){return x.id!==b.id;});}
  persist();changed();schedulePush();
}
function removeBullets(ids){
  var used={};lib.loads.forEach(function(l){used[l.bulletId]=1;});
  var gone=0;
  lib.bullets.slice().forEach(function(b){if(ids.indexOf(b.id)>=0&&!used[b.id]){tomb(b.source==='library'?'library':'bullet',b.id);gone++;}});
  lib.bullets=lib.bullets.filter(function(b){return !(ids.indexOf(b.id)>=0&&!used[b.id]);});
  persist();changed();schedulePush();return gone;
}
// ---------- whole records (the Skyward-style screens edit these directly) ----------
var LISTS={rifle:'rifles',load:'loads',bullet:'bullets',card:'cards',shot:'shots'};
// Save one record as it is (new or edited). Returns its id.
function saveRecord(kind,rec){
  var key=LISTS[kind];if(!key)throw new Error('unknown kind '+kind);
  var rc=clone(rec);if(!rc.id)rc.id=uuid();
  var old=byId(lib[key],rc.id);
  if(stamp(lib[key],rc,old)){persist();changed();schedulePush();}
  return rc.id;
}
// Delete one record (tombstone, so other devices drop it too). A firearm takes
// its ammo with it, like in Skyward.
function removeRecord(kind,id){
  var key=LISTS[kind];if(!key)return;
  var rec=byId(lib[key],id);if(!rec)return;
  if(kind==='rifle')lib.loads.filter(function(l){return l.rifleId===id;}).forEach(function(l){tomb('load',l.id);});
  if(kind==='rifle')lib.loads=lib.loads.filter(function(l){return l.rifleId!==id;});
  tomb(kind==='bullet'&&rec.source==='library'?'library':kind,id);
  lib[key]=lib[key].filter(function(x){return x.id!==id;});
  persist();changed();schedulePush();
}

// Engine inputs for a flat profile (p.loadId = the saved load it came from, if any). Group offsets are NOT passed here: the page
// adds them to the holds in MOA exactly as before this change.
function engineGear(p){
  var b0=bullet(p.bulletId);
  var bl=bulletFromFlat(p,b0&&modelName(p.model)==='CUSTOM'?b0:null);
  if(modelName(p.model)==='CUSTOM'&&bl.dragCurve.length<2)throw new Error('Enter a drag curve (Mach:Cd pairs) or pick G1/G7.');
  var l0=p.loadId?loadRec(p.loadId):null;   // Skyward extras the flat view doesn't show
  return {
    rifle:{twistIn:n(p.twist,10),twistDirection:(''+p.twistDir)==='-1'?'LEFT':'RIGHT',sightHeightIn:n(p.sh,1.75),
           sightOffsetIn:n(((l0&&rifle(l0.rifleId))||{}).sightOffsetIn,0)},
    load:{muzzleVelocityFps:n(p.mv,2700),zeroRangeYd:n(p.zero,100),powderTempF:n(p.powderTempF,59),
          velocityPerDegFps:n(p.mvPerF,0),dragScaleFactor:n(p.dragScale,1),zeroHeightIn:0,zeroOffsetIn:0,
          zeroAtmosphere:l0&&l0.zeroAtmosphere?l0.zeroAtmosphere:null,
          bcSeason:l0&&l0.bcSeason?l0.bcSeason:null},
    bullet:Object.assign(bl,{seasonalBcPoints:(b0&&b0.seasonalBcPoints)||{}})
  };
}
// One solve in the shape of the old ESP32 /solve JSON. p = solverParams().
function solve(p){
  if(!root.SkyEngine)return {ok:false,err:'engine'};
  var g;try{g=engineGear(p);}catch(e){return {ok:false,err:e.message};}
  return root.SkyEngine.solveOne(g.rifle,g.load,g.bullet,{
    distanceYd:n(p.range,0),windSpeed1Mph:n(p.windmph,0),windAngleDeg:n(p.windrel,90),
    spinDrift:(''+p.spin)!=='0',aeroJump:(''+p.jump)!=='0',spinDriftModel:p.spinModel||'LITZ_CLASSIC',
    coriolis:(''+p.earth)==='1',latitudeDeg:n(p.lat,0),azimuthDeg:n(p.az,0),
    lookAngleDeg:n(p.look,0),cantDeg:n(p.cant,0),targetSpeedMph:n(p.tgtMph,0),windSpeed2Mph:n(p.wind2,0),
    localGravity:(''+p.localG)==='1',
    atmosphere:atmoOf(p)});
}
// Atmosphere for a flat profile. Live base-station pressure is station
// (absolute) pressure; a typed one can be a sea-level forecast (presAbs=0).
function atmoOf(p){
  return {temperatureF:n(p.tempF,59),pressureInHg:n(p.pres,29.92),pressureIsAbsolute:(''+p.presAbs)!=='0',
          altitudeFt:n(p.alt,0),humidityPct:n(p.hum,0)};
}
function sensitivity(p,rangeYd){
  if(!root.SkyEngine)return {ok:false,err:'engine'};
  var g;try{g=engineGear(p);}catch(e){return {ok:false,err:e.message};}
  return root.SkyEngine.windSensitivity(g.rifle,g.load,g.bullet,atmoOf(p),n(rangeYd,n(p.range,0)));
}

// ---------- base station sync ----------
var pushTimer=null,syncing=false,again=false;
function setStatus(s,err){meta.status=s;meta.err=err||'';persist();changed();}
function fetchT(url,opt,ms){
  var ctl=root.AbortController?new AbortController():null;
  var t=setTimeout(function(){if(ctl)ctl.abort();},ms||6000);
  opt=Object.assign({cache:'no-store'},opt||{});if(ctl)opt.signal=ctl.signal;
  return fetch(url,opt).then(function(r){clearTimeout(t);return r;},function(e){clearTimeout(t);throw e;});
}
// gzip bytes served without a gzip header (e.g. after a power cut while saving)
function gunzipIfNeeded(buf){
  var u=new Uint8Array(buf);
  if(u.length>2&&u[0]===0x1f&&u[1]===0x8b&&root.DecompressionStream&&root.Response){
    return new Response(new Blob([u]).stream().pipeThrough(new DecompressionStream('gzip'))).text().catch(function(){return '';});
  }
  return new TextDecoder().decode(u);
}
function gzipBlob(text){
  if(!root.CompressionStream||!root.Response)return Promise.resolve({blob:new Blob([text],{type:'application/json'}),gz:false});
  try{
    var cs=new Blob([text]).stream().pipeThrough(new CompressionStream('gzip'));
    return new Response(cs).blob().then(function(b){return {blob:b,gz:true};},function(){return {blob:new Blob([text],{type:'application/json'}),gz:false};});
  }catch(e){return Promise.resolve({blob:new Blob([text],{type:'application/json'}),gz:false});}
}
function schedulePush(){if(pushTimer)clearTimeout(pushTimer);pushTimer=setTimeout(function(){pushTimer=null;sync();},800);}
// Pull the base station copy, merge, push the result back if it differs.
function sync(){
  if(syncing){again=true;return Promise.resolve();}
  syncing=true;again=false;
  var remoteHash=null;
  return fetchT('/gear/info').then(function(r){return r.json();}).then(function(info){
    if(!info.ok)throw new Error('base station flash unavailable');
    var pull=(info.ver!==meta.serverVer&&info.bytes>0)?fetchT('/gear',{},20000).then(function(r){
      if(r.status===204)return null;
      if(!r.ok)throw new Error('read failed ('+r.status+')');
      var v=parseInt(r.headers.get('X-Gear-Ver'),10);
      return r.arrayBuffer().then(gunzipIfNeeded).then(function(t){return {text:t,ver:isFinite(v)?v:info.ver};});
    }):Promise.resolve(null);
    return pull.then(function(got){
      var base=info.ver;
      var res=null;
      if(got){try{res=parseFile(got.text);}catch(e){
        // The base's copy is unreadable (not a format or version we don't know: those are
        // kept). Nobody can read it, so this phone's gear replaces it.
        if(!e.unreadable)throw e;base=got.ver;meta.repairedBase=true;}}
      if(res){
        remoteHash=hash(canon(res.lib));
        var before=canon(lib);mergeInto(lib,res.lib,false);base=got.ver;
        if(canon(lib)!==before)changed();
      }else if(info.bytes===0){remoteHash=hash(canon(emptyLib()));}
      else if(info.ver===meta.serverVer){remoteHash=meta.serverHash||null;}
      meta.serverVer=base;persist();
      if(remoteHash!==null&&remoteHash===hash(canon(lib))){meta.serverHash=remoteHash;return 'same';}
      if(!lib.loads.length&&!lib.bullets.length&&!lib.rifles.length&&!(lib.deleted||[]).length)return 'same';
      // A file the base already refused is not sent again until the gear changes:
      // each try ties up the base's web server (and live wind) for several seconds.
      var sent=hash(canon(lib)),mine=sent+'@'+(info.max||0);   // retried if the gear or the base's limit changes
      if(meta.tooBig===mine)throw new Error(meta.tooBigMsg||'gear file too big for the base station');
      var text=exportText('master');
      return gzipBlob(text).then(function(z){
        function refuse(){meta.tooBig=mine;meta.tooBigMsg='gear file too big for the base station ('+Math.ceil(z.blob.size/1024)+' KB, limit '+Math.floor((info.max||0)/1024)+' KB)';persist();throw new Error(meta.tooBigMsg);}
        if(info.max&&z.blob.size>info.max)refuse();
        var fd=new FormData();fd.append('file',z.blob,'gear.json');
        return fetchT('/gear?base='+base+'&gz='+(z.gz?1:0),{method:'POST',body:fd},30000).then(function(r){
          return r.json().then(function(j){
            if(r.status===409){again=true;meta.serverVer=-1;return 'conflict';}
            if(!j.ok&&j.err==='too_big')refuse();
            if(!j.ok)throw new Error(j.err==='flash_full'?'base station flash full':'save failed ('+(j.err||r.status)+')');
            // record what was SENT: an edit made during the upload still needs pushing
            meta.serverVer=j.ver;meta.serverHash=sent;delete meta.tooBig;delete meta.tooBigMsg;
            if(hash(canon(lib))!==sent)again=true;
            return 'pushed';
          });
        });
      });
    });
  }).then(function(){
    meta.lastSync=now();setStatus('synced with base station');
  },function(e){
    setStatus('saved on this phone only',(e&&e.name==='AbortError')?'base station not answering':(e&&e.message)||'base station unreachable');
  }).then(function(){syncing=false;if(again){again=false;return sync();}});
}

// ---------- first run: convert old Coriolis profiles ----------
function migrate(){
  if(lsGet(KEY)!==null)return 0;
  var old=null;try{old=JSON.parse(lsGet('profiles')||'null');}catch(e){}
  var c=0;
  if(old&&typeof old==='object'){
    var res;try{res=parseFile(JSON.stringify(old));}catch(e){res=null;}
    if(res){mergeInto(lib,res.lib,true);c=res.lib.loads.length;
      var act=lsGet('activeProfile');
      if(act)lsSet('activeLoadId',stableId('coriolis-load:'+act));}
  }
  persist();
  return c;
}

load();
// Another tab on this phone saved gear (same localStorage, different memory):
// merge it in, so this tab's next persist() can't write over that tab's edit.
if(root.addEventListener)root.addEventListener('storage',function(e){
  if(!e||e.key!==KEY||!e.newValue)return;
  var inc;try{inc=normalize(JSON.parse(e.newValue));}catch(x){return;}
  var before=canon(lib);mergeInto(lib,inc,false);var after=canon(lib);
  if(after!==canon(inc))lsSet(KEY,JSON.stringify(lib));   // we hold something that tab lacks
  if(after!==before)changed();
});
var migrated=migrate();
root.Gear={
  lib:function(){return lib;},meta:function(){return meta;},
  loads:loadsSorted,label:label,rifle:rifle,load:loadRec,bullet:bullet,bullets:function(){return lib.bullets;},
  flat:flat,saveFlat:saveFlat,removeLoad:removeLoad,removeBullets:removeBullets,
  rifles:function(){return lib.rifles;},saveRecord:saveRecord,removeRecord:removeRecord,atmoOf:atmoOf,uuid:uuid,
  importText:importText,exportText:exportText,parseFile:parseFile,mergeInto:mergeInto,
  engineGear:engineGear,rifleShare:rifleShare,solve:solve,sensitivity:sensitivity,parseBands:parseBands,parseCurve:parseCurve,curveToText:curveToText,
  sync:sync,onChange:function(f){listeners.push(f);},migrated:migrated,stableId:stableId,
  _reset:function(){lib=emptyLib();meta={serverVer:-1,lastSync:0,status:'',err:''};persist();}
};
})(typeof window!=='undefined'?window:this);
)GEARJS";

#endif // GEAR_JS_H
