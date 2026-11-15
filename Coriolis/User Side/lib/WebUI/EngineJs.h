#ifndef ENGINE_JS_H
#define ENGINE_JS_H

/**
 * EngineJs.h  --  the ballistic solver that runs on the PHONE   (served as /engine.js)
 * =====================================================================================
 * JavaScript port of the Skyward Ballistics engine (longrangelab/skyward-ballistics,
 * app/src/main/java/com/longrange/dragcurve/engine/Engine.kt + Truing.kt at 72b788e),
 * plus Coriolis's per-segment wind sensitivity sweep (from Ballistics.h).
 *
 * Why on the phone: every /solve on the ESP32 is 100-400 ms of software double
 * math that stalls the radio loop, and the sensitivity sweep is 2-4 s. In the
 * browser the same work is a few ms, so the base station only serves data.
 *
 * Model: 3-DOF point mass, RK4 (dt 0.5 ms), G1/G7 drag with velocity-banded BCs
 * or a custom Cd-vs-Mach curve, humid-air density, optional Coriolis/Eotvos,
 * spin drift (three formulas) and Litz aerodynamic jump added as corrections.
 * Same physics and tables as Ballistics.h: test/engine_js/ checks the two agree.
 *
 * Units: imperial in (in, gr, ft/s, yd, degF, inHg, mph), angles out in mil.
 * Signs: windage/hold + = RIGHT, come-up + = UP, wind angle 90 = from the right.
 *
 * Keep this file plain ES5-ish JavaScript (no build step, old phones).
 */

static const char ENGINE_JS[] PROGMEM = R"ENGINEJS(
(function(root){
'use strict';
var FT=0.3048,YD=0.9144,INCH=0.0254,GRAIN=6.479891e-5,G0=9.80665,MPH=0.44704;
var LB_IN2_TO_KG_M2=703.06958,OMEGA_EARTH=7.2921159e-5,MOA_PER_MIL=3.43774677;

// Standard drag tables, flat [mach,cd,mach,cd,...]. Identical to Ballistics.h.
var G7=[0.00,0.1198,0.05,0.1197,0.10,0.1196,0.15,0.1194,0.20,0.1193,0.25,0.1194,0.30,0.1194,0.35,0.1194,0.40,0.1193,0.45,0.1193,
0.50,0.1194,0.55,0.1193,0.60,0.1194,0.65,0.1197,0.70,0.1202,0.725,0.1207,0.75,0.1215,0.775,0.1226,0.80,0.1242,0.825,0.1266,
0.85,0.1306,0.875,0.1368,0.90,0.1464,0.925,0.1660,0.95,0.2054,0.975,0.2993,1.00,0.3803,1.025,0.4015,1.05,0.4043,1.075,0.4034,
1.10,0.4014,1.125,0.3987,1.15,0.3955,1.20,0.3884,1.25,0.3810,1.30,0.3732,1.35,0.3657,1.40,0.3580,1.50,0.3440,1.55,0.3376,
1.60,0.3315,1.65,0.3260,1.70,0.3209,1.75,0.3160,1.80,0.3117,1.85,0.3078,1.90,0.3042,1.95,0.3010,2.00,0.2980,2.05,0.2951,
2.10,0.2922,2.15,0.2892,2.20,0.2864,2.25,0.2835,2.30,0.2807,2.35,0.2779,2.40,0.2752,2.45,0.2725,2.50,0.2697,2.55,0.2670,
2.60,0.2643,2.65,0.2615,2.70,0.2588,2.75,0.2561,2.80,0.2533,2.85,0.2506,2.90,0.2479,2.95,0.2451,3.00,0.2424,3.10,0.2368,
3.20,0.2313,3.30,0.2258,3.40,0.2205,3.50,0.2154,3.60,0.2106,3.70,0.2060,3.80,0.2017,3.90,0.1975,4.00,0.1935,4.20,0.1861,
4.40,0.1793,4.60,0.1730,4.80,0.1672,5.00,0.1618];
var G1=[0.000,0.2629,0.050,0.2558,0.100,0.2487,0.150,0.2413,0.200,0.2344,0.250,0.2278,0.300,0.2214,0.350,0.2155,0.400,0.2104,
0.450,0.2061,0.500,0.2032,0.550,0.2020,0.600,0.2034,0.700,0.2165,0.725,0.2230,0.750,0.2313,0.775,0.2417,0.800,0.2546,
0.825,0.2706,0.850,0.2901,0.875,0.3136,0.900,0.3415,0.925,0.3734,0.950,0.4084,0.975,0.4448,1.000,0.4805,1.025,0.5136,
1.050,0.5427,1.075,0.5677,1.100,0.5883,1.125,0.6053,1.150,0.6191,1.200,0.6393,1.250,0.6518,1.300,0.6589,1.350,0.6621,
1.400,0.6625,1.450,0.6607,1.500,0.6573,1.550,0.6528,1.600,0.6474,1.650,0.6413,1.700,0.6347,1.750,0.6280,1.800,0.6210,
1.850,0.6141,1.900,0.6072,1.950,0.6003,2.000,0.5934,2.050,0.5867,2.100,0.5804,2.150,0.5743,2.200,0.5685,2.250,0.5630,
2.300,0.5577,2.350,0.5527,2.400,0.5481,2.450,0.5438,2.500,0.5397,2.600,0.5325,2.700,0.5264,2.800,0.5211,2.900,0.5168,
3.000,0.5133,3.100,0.5105,3.200,0.5084,3.300,0.5067,3.400,0.5054,3.500,0.5040,3.600,0.5030,3.700,0.5022,3.800,0.5016,
3.900,0.5010,4.000,0.5006,4.200,0.4998,4.400,0.4995,4.600,0.4992,4.800,0.4990,5.000,0.4988];

function lookup(t,x){
  var n=t.length/2;
  if(x<=t[0])return t[1];
  if(x>=t[2*(n-1)])return t[2*n-1];
  var lo=0,hi=n-1;
  while(hi-lo>1){var mid=(lo+hi)>>1;if(t[2*mid]<=x)lo=mid;else hi=mid;}
  var x0=t[2*lo],x1=t[2*hi];
  return t[2*lo+1]+(x-x0)/(x1-x0)*(t[2*hi+1]-t[2*lo+1]);
}
function num(v,d){v=+v;return isFinite(v)?v:d;}
function rad(d){return d*Math.PI/180;}

function localGravity(latDeg,altFt){
  var s2=Math.pow(Math.sin(rad(latDeg)),2);
  var g0=9.7803253359*(1+0.00193185265241*s2)/Math.sqrt(1-0.00669437999013*s2);
  return g0-3.086e-6*altFt*FT;
}

// Atmosphere {altitudeFt, pressureInHg, pressureIsAbsolute, temperatureF, humidityPct}
function Air(a){
  var tempK=(num(a.temperatureF,59)-32)*5/9+273.15;
  var p=num(a.pressureInHg,29.92)*3386.389;
  if(a.pressureIsAbsolute===false)p=p*Math.pow(1-2.25577e-5*num(a.altitudeFt,0)*FT,5.25588);
  var tc=tempK-273.15;
  var es=611.21*Math.exp((18.678-tc/234.5)*(tc/(257.14+tc)));
  var pv=num(a.humidityPct,0)/100*es;
  this.tempK=tempK;this.stationPa=p;
  this.density=(p-pv)/(287.058*tempK)+pv/(461.495*tempK);
  this.speedOfSound=Math.sqrt(1.4*287.058*tempK);
}

function seasonFor(tempF){return tempF<30?'cold':(tempF>90?'hot':'mild');}

// Drag for a bullet (+ optional load overrides), as Skyward's Drag.of().
function Drag(bullet,load,tempF,scaleMul){
  load=load||{};
  var model=load.dragModelOverride||bullet.dragModel||'G7';
  var season=load.bcSeason?(load.bcSeason==='auto'?seasonFor(tempF):load.bcSeason):null;
  var rows=(season&&bullet.seasonalBcPoints&&bullet.seasonalBcPoints[season])||load.bcPointsOverride||bullet.bcPoints||[];
  rows=rows.slice().sort(function(a,b){return b.minVelFps-a.minVelFps;});
  var curve=(bullet.dragCurve||[]).slice().sort(function(a,b){return a.mach-b.mach;});
  var ct=[];for(var i=0;i<curve.length;i++){ct.push(+curve[i].mach,+curve[i].cd);}
  if(model!=='CUSTOM'&&!rows.length)throw new Error('no BC rows');
  if(model==='CUSTOM'&&curve.length<2)throw new Error('drag curve needs at least 2 points');
  for(i=0;i<rows.length;i++)if(!(rows[i].bc>0))throw new Error('BC must be above 0');
  var aOverM=Math.PI/4*Math.pow(bullet.diameterIn*INCH,2)/(bullet.weightGr*GRAIN);
  var scale=num(load.dragScaleFactor,1)*(scaleMul||1);
  var table=model==='G1'?G1:G7;
  function bcAt(vFps){
    if(rows.length===1)return rows[0].bc;
    for(var i=0;i<rows.length;i++)if(vFps>=rows[i].minVelFps)return rows[i].bc;
    return rows[rows.length-1].bc;
  }
  this.k=function(rho,airSpeed,groundSpeed,sos){
    var mach=airSpeed/sos;
    if(model==='CUSTOM')return scale*0.5*rho*lookup(ct,mach)*aOverM*airSpeed;
    var bc=bcAt(groundSpeed/FT)*LB_IN2_TO_KG_M2;
    return scale*rho*lookup(table,mach)*Math.PI/8/bc*airSpeed;
  };
}

function muzzleVelocity(load,tempF){
  return num(load.muzzleVelocityFps,0)+num(load.velocityPerDegFps,0)*(tempF-num(load.powderTempF,59));
}
function estimateLength(b){var r=b.weightGr/Math.pow(b.diameterIn,3)/1000;return b.diameterIn*1.22*Math.pow(r,0.73);}
function bulletLength(b){return (b.lengthIn>0)?b.lengthIn:estimateLength(b);}
function stability(bullet,rifle,mvFps,atmo){
  var d=bullet.diameterIn,len=bulletLength(bullet)/d,t=rifle.twistIn/d;
  if(!(t>0))return 0;
  var sg=30*bullet.weightGr/(t*t*Math.pow(d,3)*len*(1+len*len));
  var air=new Air(atmo);
  return sg*Math.pow(mvFps/2800,1/3)*(air.tempK*9/5/518.67)*(29.92/(air.stationPa/3386.389));
}

var DT=0.0005;

// RK4 from the muzzle; returns the state [x,y,z,vx,vy,vz,t] at each target
// distance along the line of sight (interpolated, not first step past).
// windFn (optional) = function(x) -> crosswind z-velocity (m/s) for that x.
function integrate(drag,mv,sightH,sightOff,air,elevRad,windRad,wind,omega,targets,look,g,windFn){
  var rho=air.density,sos=air.speedOfSound,v0=mv*FT,cl=Math.cos(look),sl=Math.sin(look);
  var s=[sightH*sl,-sightH*cl,-sightOff,v0*Math.cos(elevRad)*Math.cos(windRad),v0*Math.sin(elevRad),v0*Math.cos(elevRad)*Math.sin(windRad),0];
  var out=[],idx=0,j;
  while(idx<targets.length&&targets[idx]<=0){out.push(s.slice());idx++;}
  var k1=new Array(6),k2=new Array(6),k3=new Array(6),k4=new Array(6),tmp=new Array(7),ns=new Array(7);
  var w0=wind[0],w1=wind[1],w2=wind[2];
  function deriv(st,d){
    var wz=windFn?windFn(st[0]):w2;
    var rvx=st[3]-w0,rvy=st[4]-w1,rvz=st[5]-wz;
    var a=Math.sqrt(rvx*rvx+rvy*rvy+rvz*rvz),gr=Math.sqrt(st[3]*st[3]+st[4]*st[4]+st[5]*st[5]);
    var k=drag.k(rho,a,gr,sos);
    var ax=-k*rvx,ay=-k*rvy-g,az=-k*rvz;
    if(omega){
      ax+=-2*(omega[1]*st[5]-omega[2]*st[4]);
      ay+=-2*(omega[2]*st[3]-omega[0]*st[5]);
      az+=-2*(omega[0]*st[4]-omega[1]*st[3]);
    }
    d[0]=st[3];d[1]=st[4];d[2]=st[5];d[3]=ax;d[4]=ay;d[5]=az;
  }
  var h=DT;
  while(idx<targets.length&&s[6]<30){
    deriv(s,k1);for(j=0;j<6;j++)tmp[j]=s[j]+0.5*h*k1[j];
    deriv(tmp,k2);for(j=0;j<6;j++)tmp[j]=s[j]+0.5*h*k2[j];
    deriv(tmp,k3);for(j=0;j<6;j++)tmp[j]=s[j]+h*k3[j];
    deriv(tmp,k4);
    for(j=0;j<6;j++)ns[j]=s[j]+h/6*(k1[j]+2*k2[j]+2*k3[j]+k4[j]);
    ns[6]=s[6]+h;
    var a0=s[0]*cl+s[1]*sl,a1=ns[0]*cl+ns[1]*sl;
    while(idx<targets.length&&a1>=targets[idx]){
      var fr=(targets[idx]-a0)/(a1-a0),o=new Array(7);
      for(j=0;j<7;j++)o[j]=s[j]+fr*(ns[j]-s[j]);
      out.push(o);idx++;
    }
    var t=s;s=ns;ns=t;
    if(s[3]<=0)break;
  }
  while(out.length<targets.length)out.push(null);   // never reached that range
  return out;
}

function gravityFor(shot){return shot.localGravity?localGravity(num(shot.latitudeDeg,0),num(shot.atmosphere&&shot.atmosphere.altitudeFt,0)):G0;}

function zeroAngles(rifle,load,drag,mv,atmo,g){
  var air=new Air(atmo),zx=num(load.zeroRangeYd,100)*YD,noWind=[0,0,0];
  function yAt(th){
    var s=integrate(drag,mv,num(rifle.sightHeightIn,1.5)*INCH,0,air,th,0,noWind,null,[zx],0,g)[0];
    if(!s)return NaN;
    return s[1]-num(load.zeroHeightIn,0)*INCH;
  }
  var t1=0,t2=0.004,y1=yAt(t1),y2=yAt(t2),elev=NaN;
  for(var it=0;it<30;it++){
    if(!isFinite(y1)||!isFinite(y2)||Math.abs(y2-y1)<1e-14)break;
    var t3=t2-y2*(t2-t1)/(y2-y1);
    t1=t2;y1=y2;t2=t3;y2=yAt(t2);
    if(Math.abs(y2)<1e-7){elev=t2;break;}
  }
  if(isNaN(elev)){
    var lo=-0.01,hi=0.05;
    for(it=0;it<60;it++){var m=0.5*(lo+hi);if(yAt(m)>0)hi=m;else lo=m;}
    elev=0.5*(lo+hi);
    // Skyward accepts the bisection result; Coriolis refuses an unzeroed rifle.
    if(!(Math.abs(yAt(elev))<0.005))throw new Error('could not zero');
  }
  return {elevRad:elev,windRad:(num(rifle.sightOffsetIn,0)*INCH+num(load.zeroOffsetIn,0)*INCH)/zx};
}

function earth(shot){
  var lat=rad(num(shot.latitudeDeg,0)),az=rad(num(shot.azimuthDeg,0));
  return [OMEGA_EARTH*Math.cos(lat)*Math.cos(az),OMEGA_EARTH*Math.sin(lat),-OMEGA_EARTH*Math.cos(lat)*Math.sin(az)];
}

function validate(rifle,load,bullet){
  if(!(bullet.diameterIn>0))throw new Error('bullet diameter must be above 0');
  if(!(bullet.weightGr>0))throw new Error('bullet weight must be above 0');
  if(!(load.muzzleVelocityFps>0))throw new Error('muzzle velocity must be above 0');
  if(!(load.zeroRangeYd>0))throw new Error('zero range must be above 0');
}

// Full solution, as Skyward's Engine.solve(). shot = {distanceYd, lookAngleDeg,
// cantDeg, targetSpeedMph, coriolis, latitudeDeg, azimuthDeg, atmosphere,
// windSpeed1Mph, windSpeed2Mph, windAngleDeg, spinDrift, aeroJump,
// spinDriftModel: 'LITZ_CLASSIC' (Coriolis default) | 'LITZ' | 'MEASURED', localGravity}
function solve(rifle,load,bullet,shot,opt){
  opt=opt||{};
  validate(rifle,load,bullet);
  var atmo=shot.atmosphere||{};
  var tempF=num(atmo.temperatureF,59);
  var zeroDrag=new Drag(bullet,load,tempF,1);
  var bcFactor=num(opt.bcFactor,1);
  var drag=bcFactor===1?zeroDrag:new Drag(bullet,load,tempF,1/bcFactor);
  var mv=muzzleVelocity(load,tempF)+num(opt.mvShiftFps,0);
  var zeroAtmo=load.zeroAtmosphere||atmo;
  var mvZero=muzzleVelocity(load,num(zeroAtmo.temperatureF,59));
  var g=gravityFor(shot);
  var zero=zeroAngles(rifle,load,zeroDrag,mvZero,zeroAtmo,g);
  var dist=num(shot.distanceYd,0);
  var maxR=num(opt.maxRangeYd,dist),step=num(opt.stepYd,50);
  var ranges=[],r;
  if(step>0)for(r=0;r<=maxR+1e-9;r+=step)ranges.push(+r.toFixed(6));
  (opt.extraRangesYd||[dist]).forEach(function(x){if(x>=0)ranges.push(x);});
  ranges=ranges.filter(function(x,i,a){return a.indexOf(x)===i;}).sort(function(a,b){return a-b;});

  var air=new Air(atmo);
  var look=rad(num(shot.lookAngleDeg,0)),wa=rad(num(shot.windAngleDeg,90));
  function windVec(mph){return [-mph*MPH*Math.cos(wa),0,-mph*MPH*Math.sin(wa)];}
  var sightH=num(rifle.sightHeightIn,1.5)*INCH,sightOff=num(rifle.sightOffsetIn,0)*INCH;
  var targets=ranges.map(function(x){return x*YD;});
  var cor=shot.coriolis?earth(shot):null;
  function fly(w,tg){
    var a=integrate(drag,mv,sightH,sightOff,air,zero.elevRad+look,zero.windRad,w,cor,tg,look,g);
    var b=cor?integrate(drag,mv,sightH,sightOff,air,zero.elevRad+look,zero.windRad,w,null,tg,look,g):a;
    return a.map(function(s,i){
      var n=b[i];if(!s||!n)return null;
      return {t:s[6],perp:-s[0]*Math.sin(look)+s[1]*Math.cos(look),z:s[2],
              speed:Math.sqrt(s[3]*s[3]+s[4]*s[4]+s[5]*s[5]),
              perpNo:-n[0]*Math.sin(look)+n[1]*Math.cos(look),zNo:n[2]};
    });
  }
  var w1=num(shot.windSpeed1Mph,0),w2=num(shot.windSpeed2Mph,0);
  var main=fly(windVec(w1),targets);
  var second=w2!==0?fly(windVec(w2),targets):null;
  var calm=(w1!==0&&!opt.skipCalm)?fly([0,0,0],targets):main;   // only windDriftIn needs it

  var sg0=stability(bullet,rifle,mv,atmo);
  var twistSign=rifle.twistDirection==='LEFT'?-1:1;
  var crossMph=w1*Math.sin(wa);
  var lCal=bulletLength(bullet)/bullet.diameterIn;
  var ajMil=(shot.aeroJump!==false&&sg0>0)?crossMph*twistSign*(0.01*sg0-0.0024*lCal+0.032)/MOA_PER_MIL:0;
  var model=shot.spinDriftModel||'LITZ_CLASSIC';
  function litz(t){return 1.25*(sg0+1.2)*Math.pow(t,1.83);}
  var zeroSpinIn=0;
  if(shot.spinDrift!==false&&model==='LITZ'&&load.zeroRangeYd>0){
    var zp=fly([0,0,0],[load.zeroRangeYd*YD])[0];
    if(zp)zeroSpinIn=litz(zp.t);
  }
  var cant=rad(num(shot.cantDeg,0));
  var rows=[];
  for(var i=0;i<ranges.length;i++){
    var p=main[i],c=calm[i];r=ranges[i];
    if(!p||!c)break;                       // bullet stopped short of this range
    var rangeM=r*YD,v=p.speed;
    var sd=0;
    if(shot.spinDrift!==false&&r>0&&sg0>0){
      if(model==='MEASURED')sd=2.184*Math.pow(sg0,0.537)*Math.pow(p.t,1.920);
      else if(model==='LITZ')sd=litz(p.t)-zeroSpinIn*r/load.zeroRangeYd;
      else sd=litz(p.t);
      sd*=twistSign;
    }
    var ajIn=r>0?ajMil/1000*rangeM/INCH:0;
    var pathIn=p.perp/INCH+ajIn,windIn=p.z/INCH+sd;
    if(cant!==0&&r>0){var rise=-pathIn;windIn+=rise*Math.sin(cant);pathIn-=rise*(1-Math.cos(cant));}
    var toMil=function(inches){return r>0?Math.atan2(inches*INCH,rangeM)*1000:0;};
    var s2=second?second[i]:null;
    rows.push({
      rangeYd:r,pathIn:pathIn,pathMil:toMil(pathIn),comeUpMil:-toMil(pathIn),
      windageIn:windIn,windageMil:-toMil(windIn),
      windage2Mil:s2?-toMil(s2.z/INCH+sd):null,
      windDriftIn:(p.z-c.z)/INCH,spinDriftIn:sd,aeroJumpMil:r>0?ajMil:0,
      coriolisVertIn:cor?(p.perp-p.perpNo)/INCH:0,coriolisHorizIn:cor?(p.z-p.zNo)/INCH:0,
      leadMil:toMil(num(shot.targetSpeedMph,0)*MPH*p.t/INCH),
      velocityFps:v/FT,mach:v/air.speedOfSound,
      energyFtLb:0.5*bullet.weightGr*GRAIN*v*v/1.3558179483,
      tofS:p.t,stability:sg0*Math.pow(v/(mv*FT),-1.198),
      dropIn:-p.perp/INCH,impactZIn:p.z/INCH
    });
  }
  function crossing(m){
    for(var i=0;i<rows.length-1;i++){var a=rows[i],b=rows[i+1];
      if(a.mach>=m&&b.mach<m)return a.rangeYd+(a.mach-m)/(a.mach-b.mach)*(b.rangeYd-a.rangeYd);}
    return null;
  }
  return {rows:rows,muzzleVelocityFps:mv,zeroAngleMil:zero.elevRad*1000,stabilityMuzzle:sg0,
          transonicYd:crossing(1.2),subsonicYd:crossing(1.0),
          at:function(yd){var best=null;rows.forEach(function(x){if(!best||Math.abs(x.rangeYd-yd)<Math.abs(best.rangeYd-yd))best=x;});return best;}};
}

// One range, in the shape the old ESP32 /solve returned, so every Coriolis
// feature can switch over without changing what it reads.
function solveOne(rifle,load,bullet,shot){
  try{
    var sol=solve(rifle,load,bullet,shot,{stepYd:0,extraRangesYd:[shot.distanceYd],skipCalm:true});
    var row=null;sol.rows.forEach(function(x){if(Math.abs(x.rangeYd-shot.distanceYd)<1e-6)row=x;});
    if(!row||!(shot.distanceYd>0))return {ok:false,err:'range'};
    return {ok:true,elevMil:row.comeUpMil,elevMOA:row.comeUpMil*MOA_PER_MIL,
            windMil:row.windageMil,windMOA:row.windageMil*MOA_PER_MIL,
            spinDriftIn:row.spinDriftIn,aeroJumpMOA:row.aeroJumpMil*MOA_PER_MIL,
            dropIn:row.dropIn,windDriftIn:row.impactZIn,tof:row.tofS,vRemain:row.velocityFps,
            sg:sol.stabilityMuzzle,mach:row.mach,energy:row.energyFtLb,
            transonicYd:sol.transonicYd,subsonicYd:sol.subsonicYd};
  }catch(e){return {ok:false,err:String(e&&e.message||e)};}
}

// Crosswind sensitivity along the path (Coriolis Ballistics::windSensitivity):
// zero once, then for each ~25 yd segment add 1 mph of crosswind in that
// segment only and measure the lateral impact shift at the target.
// Returns {ok, coef:[in/mph per segment, muzzle->target], segYd, rangeYd}.
function windSensitivity(rifle,load,bullet,atmo,rangeYd){
  try{
    validate(rifle,load,bullet);
    if(!(rangeYd>0))return {ok:false};
    var tempF=num(atmo.temperatureF,59),drag=new Drag(bullet,load,tempF,1);
    var mv=muzzleVelocity(load,tempF),g=G0,air=new Air(atmo);
    var zero=zeroAngles(rifle,load,drag,muzzleVelocity(load,num((load.zeroAtmosphere||atmo).temperatureF,59)),load.zeroAtmosphere||atmo,g);
    var nSeg=Math.round(rangeYd/25);if(nSeg<4)nSeg=4;if(nSeg>64)nSeg=64;
    var tgtX=rangeYd*YD,segW=tgtX/nSeg,sightH=num(rifle.sightHeightIn,1.5)*INCH;
    var prof=new Array(nSeg);for(var i=0;i<nSeg;i++)prof[i]=0;
    function windFn(x){var s=Math.floor(x/segW);if(s<0)s=0;if(s>=nSeg)s=nSeg-1;return -prof[s]*MPH;}
    function shotZ(){var s=integrate(drag,mv,sightH,0,air,zero.elevRad,0,[0,0,0],null,[tgtX],0,g,windFn)[0];return s?s[2]:NaN;}
    var z0=shotZ();if(!isFinite(z0))return {ok:false};
    var coef=[];
    for(i=0;i<nSeg;i++){prof[i]=1;var z=shotZ();prof[i]=0;coef.push(isFinite(z)?Math.abs((z-z0)/INCH):0);}
    return {ok:true,coef:coef,segYd:rangeYd/nSeg,rangeYd:rangeYd,seg:nSeg};
  }catch(e){return {ok:false,err:String(e&&e.message||e)};}
}
// Raw sensitivity at a downrange distance (Sensitivity::sensitivityAt).
function sensitivityAt(curve,yd){
  if(!curve||!curve.ok||yd<0)return 0;
  var n=curve.coef.length;if(yd>n*curve.segYd)return 0;
  var pos=yd/curve.segYd-0.5;
  if(pos<=0)return curve.coef[0];if(pos>=n-1)return curve.coef[n-1];
  var i=Math.floor(pos),f=pos-i;return curve.coef[i]*(1-f)+curve.coef[i+1]*f;
}
// GPS -> signed downrange yards along the firing azimuth (Sensitivity::projectDownrangeYd).
function projectDownrangeYd(baseLat,baseLon,azDeg,lat,lon){
  var M=111319.4908,Y=1.0936133,lat0=rad(baseLat);
  var dN=(lat-baseLat)*M*Y,dE=(lon-baseLon)*M*Math.cos(lat0)*Y,A=rad(azDeg);
  return dN*Math.cos(A)+dE*Math.sin(A);
}

// Truing (Skyward Truing.kt): fit MV or drag scale to observed come-ups.
// impacts = [{rangeYd, actualComeUpMil}]
function fitParam(start,lo0,hi0,impacts,model){
  if(!impacts.length)throw new Error('enter at least one impact');
  impacts.forEach(function(i){if(!(i.rangeYd>0))throw new Error('impact distances must be above 0');});
  function rms(pred){var s=0;pred.forEach(function(p,k){var d=p-impacts[k].actualComeUpMil;s+=d*d;});return Math.sqrt(s/impacts.length);}
  var gr=(Math.sqrt(5)-1)/2,lo=lo0,hi=hi0,c=hi-gr*(hi-lo),d=lo+gr*(hi-lo),fc=rms(model(c)),fd=rms(model(d));
  for(var it=0;it<40;it++){
    if(fc<fd){hi=d;d=c;fd=fc;c=hi-gr*(hi-lo);fc=rms(model(c));}
    else{lo=c;c=d;fc=fd;d=lo+gr*(hi-lo);fd=rms(model(d));}
  }
  var best=(lo+hi)/2,before=model(start),after=model(best);
  return {value:best,predictedBefore:before,predictedAfter:after,rmsBeforeMil:rms(before),rmsAfterMil:rms(after)};
}
function comeUps(rifle,load,bullet,shot,impacts){
  var ranges=impacts.map(function(i){return i.rangeYd;}),mx=Math.max.apply(null,ranges);
  var s=Object.assign({},shot,{distanceYd:mx});
  var sol=solve(rifle,load,bullet,s,{maxRangeYd:mx,stepYd:mx,extraRangesYd:ranges});
  return ranges.map(function(r){var row=null;sol.rows.forEach(function(x){if(Math.abs(x.rangeYd-r)<1e-6)row=x;});return row?row.comeUpMil:NaN;});
}
function trueMuzzleVelocity(rifle,load,bullet,shot,impacts){
  var t=num(shot.atmosphere&&shot.atmosphere.temperatureF,59),mv0=muzzleVelocity(load,t);
  function loadFor(v){return Object.assign({},load,{muzzleVelocityFps:v-num(load.velocityPerDegFps,0)*(t-num(load.powderTempF,59))});}
  var res=fitParam(mv0,mv0*0.8,mv0*1.2,impacts,function(v){return comeUps(rifle,loadFor(v),bullet,shot,impacts);});
  res.loadMuzzleVelocityFps=loadFor(res.value).muzzleVelocityFps;
  return res;
}
function trueDragScale(rifle,load,bullet,shot,impacts){
  var s0=num(load.dragScaleFactor,1);
  return fitParam(s0,s0*0.7,s0*1.3,impacts,function(s){return comeUps(rifle,Object.assign({},load,{dragScaleFactor:s}),bullet,shot,impacts);});
}

var api={solve:solve,solveOne:solveOne,windSensitivity:windSensitivity,sensitivityAt:sensitivityAt,
  projectDownrangeYd:projectDownrangeYd,stability:stability,muzzleVelocity:muzzleVelocity,
  estimateLength:estimateLength,localGravity:localGravity,trueMuzzleVelocity:trueMuzzleVelocity,
  trueDragScale:trueDragScale,Air:Air,MOA_PER_MIL:MOA_PER_MIL,G1:G1,G7:G7,lookup:lookup,
  setDt:function(d){DT=d;}};
root.SkyEngine=api;
if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof window!=='undefined'?window:this);
)ENGINEJS";

#endif // ENGINE_JS_H
