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
 * >>> ACCURACY WARNING <<<
 * The G1/G7 Cd tables below are approximate standard curves. Before trusting
 * any output for real dope, replace them with the official JBM/Litz values and
 * validate against a known-good solver (Applied Ballistics, Hornady, JBM).
 * A solver that is "close" is more dangerous than none, because it looks right.
 */

#include <math.h>

// ---------------------------------------------------------------------------
// Standard drag tables (Mach, Cd).  *** VERIFY against official JBM data ***
// ---------------------------------------------------------------------------
struct DragPoint { float mach; float cd; };

static const DragPoint G7_TABLE[] = {
    {0.00f,0.1198f},{0.50f,0.1197f},{0.70f,0.1194f},{0.80f,0.1193f},{0.85f,0.1194f},
    {0.90f,0.1202f},{0.95f,0.1223f},{1.00f,0.1278f},{1.05f,0.1476f},{1.10f,0.1826f},
    {1.15f,0.2223f},{1.20f,0.2586f},{1.25f,0.2874f},{1.30f,0.3072f},{1.40f,0.3261f},
    {1.50f,0.3316f},{1.60f,0.3316f},{1.80f,0.3247f},{2.00f,0.3145f},{2.20f,0.3032f},
    {2.50f,0.2853f},{3.00f,0.2604f},{3.50f,0.2416f},{4.00f,0.2287f},{5.00f,0.2100f}
};
static const DragPoint G1_TABLE[] = {
    {0.00f,0.2629f},{0.50f,0.2337f},{0.70f,0.2196f},{0.80f,0.2244f},{0.90f,0.2637f},
    {0.95f,0.3122f},{1.00f,0.3894f},{1.05f,0.4570f},{1.10f,0.5023f},{1.20f,0.5461f},
    {1.30f,0.5590f},{1.40f,0.5545f},{1.50f,0.5435f},{1.60f,0.5289f},{1.80f,0.5000f},
    {2.00f,0.4742f},{2.20f,0.4521f},{2.50f,0.4226f},{3.00f,0.3811f},{3.50f,0.3498f},
    {4.00f,0.3269f},{5.00f,0.2951f}
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
