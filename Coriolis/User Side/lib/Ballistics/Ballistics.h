#ifndef BALLISTICS_H
#define BALLISTICS_H

/**
 * Ballistics.h  --  modified point-mass ballistic solver  (STANDALONE)
 * ====================================================================
 * Pure math. No WiFi, no radio, no Arduino I/O -- so it can be unit-tested and
 * diagnosed in isolation. The web layer calls solve() and formats the result.
 *
 * Model: 3-DOF point mass (position + velocity), RK4 integration, G1/G7 drag
 * scaled by BC, air density from measured temperature + pressure. Spin drift
 * and aerodynamic jump are added as Litz empirical corrections (not a full
 * angular solve).
 *
 * Formulas (verified against published sources):
 *   Miller SG   = 30 m /(t^2 d^3 L (1+L^2)) * (Tr/519) * (29.92/P) * (V/2800)^(1/3)
 *   Spin drift  = 1.25 (SG+1.2) TOF^1.83   [inches]
 *   Aero jump   = (0.01 SG - 0.0024 Lcal + 0.032) MOA per mph of crosswind
 *
 * Drag tables (updated [see project log]):
 * G1/G7 Cd-vs-Mach below are the standard 79/84-point tables (0.025 Mach
 * spacing through transonic), traceable to Ballistic Research Laboratory
 * data as republished by JBM Ballistics; the same values are reproduced
 * across multiple independent ballistics implementations. The PREVIOUS
 * 22/24-point tables under-resolved the transonic band (Mach ~0.85-1.4) and
 * were found to understate peak Cd by up to ~66% (G7) / ~23% (G1) there --
 * that error was concentrated almost entirely in the transonic region, not
 * a uniform bias, which is why it showed up as a small elevation error (a
 * few %) alongside a much larger velocity-retention error (~9% at 1000 yd)
 * in host testing. Full trajectory (elevation, windage, velocity) was
 * cross-checked host-side against Applied Ballistics (generic G7 BC entry,
 * not a custom drag curve) and against py-ballisticcalc, matched Mach-for-
 * Mach; all three tracked within ~0.3-0.5% of each other after this change.
 *
 * >>> ACCURACY WARNING (still applies) <<<
 * This remains a standard-curve model (G1/G7), not a bullet-specific custom
 * drag model -- real bullets deviate from the standard shape, especially
 * through transonic. Re-validate against a known-good solver whenever the
 * tables, integrator, or unit conversions change. A solver that is "close"
 * is more dangerous than none, because it looks right.
 */

#include <math.h>

// ---------------------------------------------------------------------------
// Standard drag tables (Mach, Cd). Dense (0.025 Mach) through transonic.
// Source: standard G1/G7 tables (BRL origin, via JBM), cross-validated
// against Applied Ballistics + py-ballisticcalc trajectory output (see file
// header). If these are ever replaced again, re-run that same validation.
// ---------------------------------------------------------------------------
struct DragPoint { float mach; float cd; };

static const DragPoint G7_TABLE[] = {
    {0.000f,0.1198f}, {0.050f,0.1197f}, {0.100f,0.1196f}, {0.150f,0.1194f}, {0.200f,0.1193f},
    {0.250f,0.1194f}, {0.300f,0.1194f}, {0.350f,0.1194f}, {0.400f,0.1193f}, {0.450f,0.1193f},
    {0.500f,0.1194f}, {0.550f,0.1193f}, {0.600f,0.1194f}, {0.650f,0.1197f}, {0.700f,0.1202f},
    {0.725f,0.1207f}, {0.750f,0.1215f}, {0.775f,0.1226f}, {0.800f,0.1242f}, {0.825f,0.1266f},
    {0.850f,0.1306f}, {0.875f,0.1368f}, {0.900f,0.1464f}, {0.925f,0.1660f}, {0.950f,0.2054f},
    {0.975f,0.2993f}, {1.000f,0.3803f}, {1.025f,0.4015f}, {1.050f,0.4043f}, {1.075f,0.4034f},
    {1.100f,0.4014f}, {1.125f,0.3987f}, {1.150f,0.3955f}, {1.200f,0.3884f}, {1.250f,0.3810f},
    {1.300f,0.3732f}, {1.350f,0.3657f}, {1.400f,0.3580f}, {1.500f,0.3440f}, {1.550f,0.3376f},
    {1.600f,0.3315f}, {1.650f,0.3260f}, {1.700f,0.3209f}, {1.750f,0.3160f}, {1.800f,0.3117f},
    {1.850f,0.3078f}, {1.900f,0.3042f}, {1.950f,0.3010f}, {2.000f,0.2980f}, {2.050f,0.2951f},
    {2.100f,0.2922f}, {2.150f,0.2892f}, {2.200f,0.2864f}, {2.250f,0.2835f}, {2.300f,0.2807f},
    {2.350f,0.2779f}, {2.400f,0.2752f}, {2.450f,0.2725f}, {2.500f,0.2697f}, {2.550f,0.2670f},
    {2.600f,0.2643f}, {2.650f,0.2615f}, {2.700f,0.2588f}, {2.750f,0.2561f}, {2.800f,0.2533f},
    {2.850f,0.2506f}, {2.900f,0.2479f}, {2.950f,0.2451f}, {3.000f,0.2424f}, {3.100f,0.2368f},
    {3.200f,0.2313f}, {3.300f,0.2258f}, {3.400f,0.2205f}, {3.500f,0.2154f}, {3.600f,0.2106f},
    {3.700f,0.2060f}, {3.800f,0.2017f}, {3.900f,0.1975f}, {4.000f,0.1935f}, {4.200f,0.1861f},
    {4.400f,0.1793f}, {4.600f,0.1730f}, {4.800f,0.1672f}, {5.000f,0.1618f}
};
static const DragPoint G1_TABLE[] = {
    {0.000f,0.2629f}, {0.050f,0.2558f}, {0.100f,0.2487f}, {0.150f,0.2413f}, {0.200f,0.2344f},
    {0.250f,0.2278f}, {0.300f,0.2214f}, {0.350f,0.2155f}, {0.400f,0.2104f}, {0.450f,0.2061f},
    {0.500f,0.2032f}, {0.550f,0.2020f}, {0.600f,0.2034f}, {0.700f,0.2165f}, {0.725f,0.2230f},
    {0.750f,0.2313f}, {0.775f,0.2417f}, {0.800f,0.2546f}, {0.825f,0.2706f}, {0.850f,0.2901f},
    {0.875f,0.3136f}, {0.900f,0.3415f}, {0.925f,0.3734f}, {0.950f,0.4084f}, {0.975f,0.4448f},
    {1.000f,0.4805f}, {1.025f,0.5136f}, {1.050f,0.5427f}, {1.075f,0.5677f}, {1.100f,0.5883f},
    {1.125f,0.6053f}, {1.150f,0.6191f}, {1.200f,0.6393f}, {1.250f,0.6518f}, {1.300f,0.6589f},
    {1.350f,0.6621f}, {1.400f,0.6625f}, {1.450f,0.6607f}, {1.500f,0.6573f}, {1.550f,0.6528f},
    {1.600f,0.6474f}, {1.650f,0.6413f}, {1.700f,0.6347f}, {1.750f,0.6280f}, {1.800f,0.6210f},
    {1.850f,0.6141f}, {1.900f,0.6072f}, {1.950f,0.6003f}, {2.000f,0.5934f}, {2.050f,0.5867f},
    {2.100f,0.5804f}, {2.150f,0.5743f}, {2.200f,0.5685f}, {2.250f,0.5630f}, {2.300f,0.5577f},
    {2.350f,0.5527f}, {2.400f,0.5481f}, {2.450f,0.5438f}, {2.500f,0.5397f}, {2.600f,0.5325f},
    {2.700f,0.5264f}, {2.800f,0.5211f}, {2.900f,0.5168f}, {3.000f,0.5133f}, {3.100f,0.5105f},
    {3.200f,0.5084f}, {3.300f,0.5067f}, {3.400f,0.5054f}, {3.500f,0.5040f}, {3.600f,0.5030f},
    {3.700f,0.5022f}, {3.800f,0.5016f}, {3.900f,0.5010f}, {4.000f,0.5006f}, {4.200f,0.4998f},
    {4.400f,0.4995f}, {4.600f,0.4992f}, {4.800f,0.4990f}, {5.000f,0.4988f}
};

static float dragCd(int model, float mach) {
    const DragPoint* t = (model == 1) ? G1_TABLE : G7_TABLE;
    int n = (model == 1) ? (int)(sizeof(G1_TABLE)/sizeof(DragPoint))
                         : (int)(sizeof(G7_TABLE)/sizeof(DragPoint));
    if (mach <= t[0].mach)   return t[0].cd;
    if (mach >= t[n-1].mach) return t[n-1].cd;
    for (int i = 1; i < n; i++)
        if (mach < t[i].mach) {
            float f = (mach - t[i-1].mach) / (t[i].mach - t[i-1].mach);
            return t[i-1].cd + f * (t[i].cd - t[i-1].cd);
        }
    return t[n-1].cd;
}

// ---------------------------------------------------------------------------
// I/O structs (imperial in, scope corrections out)
// ---------------------------------------------------------------------------
struct BallInput {
    float mv_fps;      // muzzle velocity
    float bc;          // ballistic coefficient (in the chosen model)
    int   dragModel;   // 1 = G1, 7 = G7
    float weight_gr;   // bullet weight (grains)
    float cal_in;      // bullet diameter (inches)
    float twist_in;    // barrel twist (inches per turn)
    float blen_in;     // bullet length (inches)
    int   twistDir;    // +1 = right, -1 = left
    float sightHt_in;  // scope height over bore
    float zero_yd;     // zero range
    float range_yd;    // target range
    float tempF;       // air temperature
    float presInHg;    // station pressure (absolute)
    float wind_mph;    // wind speed
    float windRel_deg; // wind angle relative to line of fire (0 = from 12 o'clock)
    float lat_deg;     // latitude (for Coriolis/Eotvos); + = north
    float azimuth_deg; // firing azimuth (0 = north, 90 = east)
    bool  useSpinDrift;
    bool  useAeroJump;
    bool  useEarth;    // Coriolis + Eotvos
};

struct BallOutput {
    bool  ok;
    float elevMOA, elevMil;     // come-up
    float windMOA, windMil;     // wind hold (sign: + = hold right)
    float dropIn, windDriftIn, spinDriftIn, aeroJumpMOA;
    float tof, vRemain_fps, sg;
};

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------
class Ballistics {
private:
    struct S { double x,y,z,vx,vy,vz; };

    // physics constants / derived (set per solve)
    double rho, cSound, effCd_A_over_m, windx, windz, g;
    double Wx, Wy, Wz;      // Earth rotation, firing-frame components
    bool   earthOn;
    int    model;
    float  bc, sd;

    // ---- optional per-segment crosswind profile (used ONLY by windSensitivity) ----
    // When windProfileMph == nullptr the solver behaves exactly as before: the
    // uniform windx/windz set in solve() are used and nothing here is touched.
    const float* windProfileMph = nullptr;   // crosswind mph per segment, + = from the right
    double       segWidth_m     = 0.0;       // downrange width of each segment (m)
    int          nProfSeg       = 0;         // number of segments in the profile

    void deriv(const S& s, double d[6]) const {
        // Crosswind: uniform windz, unless a per-segment profile is active
        // (windSensitivity only). Profile is a pure function of downrange x.
        double wz = windz;
        if (windProfileMph && segWidth_m > 0.0 && nProfSeg > 0) {
            int seg = (int)(s.x / segWidth_m);
            if (seg < 0) seg = 0;
            if (seg >= nProfSeg) seg = nProfSeg - 1;
            wz = -(double)windProfileMph[seg] * 0.44704;   // + from right -> toward -z
        }
        double vrx = s.vx - windx, vry = s.vy, vrz = s.vz - wz;
        double vr  = sqrt(vrx*vrx + vry*vry + vrz*vrz);
        double mach = vr / cSound;
        double cd = (double)(sd / bc) * dragCd(model, (float)mach);   // effective Cd
        double k  = 0.5 * rho * cd * effCd_A_over_m;                  // = 0.5*rho*Cd*A/m
        d[0]=s.vx; d[1]=s.vy; d[2]=s.vz;
        d[3]=-k*vr*vrx; d[4]=-k*vr*vry - g; d[5]=-k*vr*vrz;
        if (earthOn) {                       // Coriolis: a = -2 (Omega x v)
            d[3] += -2.0*(Wy*s.vz - Wz*s.vy);
            d[4] += -2.0*(Wz*s.vx - Wx*s.vz);   // vertical term = Eotvos (azimuth dependent)
            d[5] += -2.0*(Wx*s.vy - Wy*s.vx);
        }
    }

    // Integrate from muzzle at launch angle theta to downrange x=targetX.
    // Fills out y,z,tof,speed at target. Returns false if it stalls.
    bool run(double theta, double v0, double sightHt_m, double targetX,
             double& yOut, double& zOut, double& tofOut, double& vOut) const {
        S s{0.0, -sightHt_m, 0.0, v0*cos(theta), v0*sin(theta), 0.0};
        double t = 0.0, dt = 0.0005;
        double d1[6],d2[6],d3[6],d4[6];
        for (int i = 0; i < 40000; i++) {
            if (s.x >= targetX) {
                yOut=s.y; zOut=s.z; tofOut=t; vOut=sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);
                return true;
            }
            S a=s; deriv(a,d1);
            S b={s.x+d1[0]*dt/2,s.y+d1[1]*dt/2,s.z+d1[2]*dt/2,s.vx+d1[3]*dt/2,s.vy+d1[4]*dt/2,s.vz+d1[5]*dt/2}; deriv(b,d2);
            S c={s.x+d2[0]*dt/2,s.y+d2[1]*dt/2,s.z+d2[2]*dt/2,s.vx+d2[3]*dt/2,s.vy+d2[4]*dt/2,s.vz+d2[5]*dt/2}; deriv(c,d3);
            S e={s.x+d3[0]*dt,  s.y+d3[1]*dt,  s.z+d3[2]*dt,  s.vx+d3[3]*dt,  s.vy+d3[4]*dt,  s.vz+d3[5]*dt};   deriv(e,d4);
            s.x  += dt/6*(d1[0]+2*d2[0]+2*d3[0]+d4[0]);
            s.y  += dt/6*(d1[1]+2*d2[1]+2*d3[1]+d4[1]);
            s.z  += dt/6*(d1[2]+2*d2[2]+2*d3[2]+d4[2]);
            s.vx += dt/6*(d1[3]+2*d2[3]+2*d3[3]+d4[3]);
            s.vy += dt/6*(d1[4]+2*d2[4]+2*d3[4]+d4[4]);
            s.vz += dt/6*(d1[5]+2*d2[5]+2*d3[5]+d4[5]);
            t += dt;
            if (s.vx < 30) return false;   // bullet stalled / went subsonic-slow
        }
        return false;
    }

public:
    BallOutput solve(const BallInput& in) {
        BallOutput o; o.ok = false;
        if (in.mv_fps <= 0 || in.bc <= 0 || in.range_yd <= 0 || in.cal_in <= 0 || in.weight_gr <= 0)
            return o;

        // --- unit conversions to SI ---
        double v0 = in.mv_fps * 0.3048;
        double m  = in.weight_gr * 6.479891e-5;          // kg
        double A  = M_PI/4.0 * pow(in.cal_in*0.0254, 2); // m^2
        double Tk = (in.tempF - 32.0)*5.0/9.0 + 273.15;
        double P  = in.presInHg * 3386.389;              // Pa
        rho = P / (287.05 * Tk);
        cSound = 20.0468 * sqrt(Tk);
        g = 9.80665;
        model = in.dragModel;
        bc = in.bc;
        sd = in.weight_gr / 7000.0f / (in.cal_in * in.cal_in);   // lb/in^2
        effCd_A_over_m = A / m;                                   // 0.5*rho*Cd*(A/m)

        // --- wind components (line-of-fire frame) ---
        double rel = in.windRel_deg * M_PI/180.0;
        double crossMph = in.wind_mph * sin(rel);       // + = from right
        double headMph  = in.wind_mph * cos(rel);       // + = headwind
        windx = -(headMph  * 0.44704);                  // headwind blows toward shooter (-x)
        windz = -(crossMph * 0.44704);                  // wind from right blows toward -z

        // --- Earth rotation in firing frame (x=downrange, y=up, z=right) ---
        earthOn = in.useEarth;
        double Om = 7.292115e-5;                         // rad/s
        double Lr = in.lat_deg * M_PI/180.0, Az = in.azimuth_deg * M_PI/180.0;
        Wx =  Om*cos(Lr)*cos(Az);
        Wy =  Om*sin(Lr);
        Wz = -Om*cos(Lr)*sin(Az);

        double sightHt_m = in.sightHt_in * 0.0254;
        double zeroX = in.zero_yd * 0.9144;
        double tgtX  = in.range_yd * 0.9144;

        // --- zero: secant search on launch angle so y = 0 at zero range ---
        double y1,z1,t1,vv1, y2,z2,t2,vv2;
        double th1 = 0.0, th2 = 0.005;                  // rad
        if (!run(th1, v0, sightHt_m, zeroX, y1,z1,t1,vv1)) return o;
        if (!run(th2, v0, sightHt_m, zeroX, y2,z2,t2,vv2)) return o;
        double theta = th2;
        for (int it = 0; it < 30; it++) {
            double denom = (y2 - y1);
            if (fabs(denom) < 1e-9) break;
            theta = th2 - y2 * (th2 - th1) / denom;
            th1 = th2; y1 = y2;
            th2 = theta;
            double zz,tt,vv;
            if (!run(th2, v0, sightHt_m, zeroX, y2,zz,tt,vv)) return o;
            if (fabs(y2) < 1e-4) break;                 // within 0.1 mm at zero
        }

        // --- fire to target with the zeroed angle ---
        double yT,zT,tof,vT;
        if (!run(theta, v0, sightHt_m, tgtX, yT,zT,tof,vT)) return o;

        // --- stability (Miller, atmosphere + velocity corrected) ---
        double Lcal = (in.blen_in > 0 && in.cal_in > 0) ? in.blen_in/in.cal_in : 0;
        double sg = 0;
        if (Lcal > 0 && in.twist_in > 0) {
            double t_cal = in.twist_in / in.cal_in;
            double Tr = in.tempF + 459.67;              // Rankine
            sg = 30.0 * in.weight_gr /
                 (t_cal*t_cal * pow(in.cal_in,3) * Lcal * (1.0 + Lcal*Lcal));
            sg *= (Tr/519.0) * (29.92/in.presInHg);     // atmosphere
            sg *= pow(in.mv_fps/2800.0, 1.0/3.0);       // velocity
        }
        o.sg = (float)sg;

        // --- linear results at target ---
        double dropIn  = -yT * 39.3701;                 // + = come-up needed (bullet below LOS)
        double windIn  =  zT * 39.3701;                 // + = impact right ... (from wind)

        // spin drift (inches, + to the right for right twist)
        double spinIn = 0;
        if (in.useSpinDrift && sg > 0)
            spinIn = in.twistDir * 1.25 * (sg + 1.2) * pow(tof, 1.83);

        // aero jump (MOA vertical, constant with range). Right-to-left wind
        // (crossMph < 0) makes a right-twist bullet go HIGH.
        double ajMOA = 0;
        if (in.useAeroJump && sg > 0) {
            double Y = 0.01*sg - 0.0024*Lcal + 0.032;   // MOA per mph crosswind
            ajMOA = in.twistDir * Y * (-crossMph);      // + = up
        }

        // --- convert to angles (scope corrections) ---
        double tgtIn = tgtX * 39.3701;
        double MOA_per_in = (10800.0/M_PI) / tgtIn;     // MOA per inch of offset at range
        double MIL_per_in = 1000.0 / tgtIn;

        double impactRightIn = windIn + spinIn;         // total horizontal impact offset
        // elevation come-up = drop correction minus aero-jump (jump up reduces come-up)
        o.elevMOA = (float)(dropIn * MOA_per_in - ajMOA);
        o.elevMil = (float)(dropIn * MIL_per_in - ajMOA/3.4377);
        // wind hold = opposite of impact (dial toward where it landed's mirror)
        o.windMOA = (float)(-impactRightIn * MOA_per_in);
        o.windMil = (float)(-impactRightIn * MIL_per_in);

        o.dropIn      = (float)dropIn;
        o.windDriftIn = (float)windIn;
        o.spinDriftIn = (float)spinIn;
        o.aeroJumpMOA = (float)ajMOA;
        o.tof         = (float)tof;
        o.vRemain_fps = (float)(vT / 0.3048);
        o.ok = true;
        return o;
    }

    // -----------------------------------------------------------------------
    // windSensitivity()  --  per-segment crosswind sensitivity along the path
    // -----------------------------------------------------------------------
    // Perturbation method: zero the rifle once (wind-independent), then for each
    // 25 yd downrange segment inject +1 mph of crosswind in THAT segment only and
    // measure the resulting lateral impact shift. The result is the "wind
    // weighting" curve -- inches of impact per mph of wind at that distance.
    // Wind near the muzzle has the whole remaining flight to act on, so the curve
    // is strongly front-loaded (muzzle segment ~ several x the target segment).
    //
    // outCoefInPerMph : caller buffer, filled with inches/mph per segment.
    // maxSeg          : size of that buffer (internally capped at 64).
    // segWidthYdOut   : (optional) receives the downrange width of one segment.
    // Returns the number of segments filled, or 0 on failure.
    //
    // NOTE: heavy call (nSeg+2 full integrations, all in double). Cache the result
    // on the receiver and only recompute when a ballistic input actually changes --
    // NEVER call this on the wind-poll cadence.
    int windSensitivity(const BallInput& in, float* outCoefInPerMph, int maxSeg,
                        float* segWidthYdOut = nullptr) {
        if (!outCoefInPerMph || maxSeg < 1) return 0;
        if (in.mv_fps <= 0 || in.bc <= 0 || in.range_yd <= 0 ||
            in.cal_in <= 0 || in.weight_gr <= 0) return 0;

        // --- atmosphere / drag setup (same as solve, but crosswind-only) ---
        double v0 = in.mv_fps * 0.3048;
        double mkg= in.weight_gr * 6.479891e-5;
        double A  = M_PI/4.0 * pow(in.cal_in*0.0254, 2);
        double Tk = (in.tempF - 32.0)*5.0/9.0 + 273.15;
        double P  = in.presInHg * 3386.389;
        rho = P / (287.05 * Tk);
        cSound = 20.0468 * sqrt(Tk);
        g = 9.80665;
        model = in.dragModel;
        bc = in.bc;
        sd = in.weight_gr / 7000.0f / (in.cal_in * in.cal_in);
        effCd_A_over_m = A / mkg;
        windx = 0.0;          // headwind is wind-independent for this sweep
        earthOn = false;      // Coriolis is a constant offset -> cancels in the delta

        double sightHt_m = in.sightHt_in * 0.0254;
        double zeroX = in.zero_yd  * 0.9144;
        double tgtX  = in.range_yd * 0.9144;

        // segment geometry (25 yd nominal, clamped to [4, 64] and buffer size)
        int nSeg = (int)lround(in.range_yd / 25.0);
        if (nSeg < 4)      nSeg = 4;
        if (nSeg > 64)     nSeg = 64;
        if (nSeg > maxSeg) nSeg = maxSeg;

        float prof[64];
        for (int i = 0; i < nSeg; i++) prof[i] = 0.0f;

        // activate the (all-zero) profile so run() reads it
        windProfileMph = prof;
        segWidth_m     = tgtX / nSeg;
        nProfSeg       = nSeg;

        int filled = 0;

        // --- zero once (crosswind = 0 -> identical to no wind) ---
        double y1,z1,t1,vv1, y2,z2,t2,vv2;
        double th1 = 0.0, th2 = 0.005;
        if (run(th1, v0, sightHt_m, zeroX, y1,z1,t1,vv1) &&
            run(th2, v0, sightHt_m, zeroX, y2,z2,t2,vv2)) {
            double theta = th2;
            for (int it = 0; it < 30; it++) {
                double denom = (y2 - y1);
                if (fabs(denom) < 1e-9) break;
                theta = th2 - y2 * (th2 - th1) / denom;
                th1 = th2; y1 = y2; th2 = theta;
                double zz,tt,vv;
                if (!run(th2, v0, sightHt_m, zeroX, y2,zz,tt,vv)) { theta = 1e9; break; }
                if (fabs(y2) < 1e-4) break;
            }

            if (theta < 1e8) {
                // baseline lateral impact (should be ~0 with no wind / no earth)
                double yb, z0, tb, vb;
                if (run(theta, v0, sightHt_m, tgtX, yb, z0, tb, vb)) {
                    for (int i = 0; i < nSeg; i++) {
                        prof[i] = 1.0f;                         // +1 mph in segment i only
                        double yy, zz, tt, vv;
                        double c = 0.0;
                        if (run(theta, v0, sightHt_m, tgtX, yy, zz, tt, vv))
                            c = (zz - z0) * 39.3701;            // inches per mph
                        prof[i] = 0.0f;
                        outCoefInPerMph[i] = (float)fabs(c);   // magnitude of influence
                    }
                    filled = nSeg;
                }
            }
        }

        // --- IMPORTANT: disarm the profile so a later solve() is untouched ---
        windProfileMph = nullptr;
        segWidth_m     = 0.0;
        nProfSeg       = 0;

        if (segWidthYdOut) *segWidthYdOut = (filled > 0) ? in.range_yd / filled : 0.0f;
        return filled;
    }
};

#endif // BALLISTICS_H
