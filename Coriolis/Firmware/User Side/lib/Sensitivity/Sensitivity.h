#ifndef SENSITIVITY_H
#define SENSITIVITY_H

/**
 * Sensitivity.h  --  wind-sensitivity zones -> per-meter weights   (STANDALONE)
 * ============================================================================
 * Phase 2. Pure math (only <math.h> + Ballistics.h) so it unit-tests and
 * diagnoses in isolation, exactly like Ballistics.h / WindStats.h. The UI/web
 * layer stays a thin viewer: it hands this module the shot parameters and meter
 * positions and gets back normalized weights.
 *
 * WHAT IT DOES
 *   1. Asks Ballistics::windSensitivity() for the crosswind-sensitivity curve
 *      along the bullet's flight path (inches of impact per mph of wind at each
 *      25 yd segment). Front-loaded: wind near the muzzle matters far more than
 *      wind near the target (validated: ~7x at 100 yd vs 900 yd on a 1000 yd shot).
 *   2. Maps each wind meter to a downrange distance -- either projected from GPS
 *      (base position + firing azimuth + meter position) or a manually entered
 *      distance -- and reads the curve there.
 *   3. Normalizes those sensitivities into weights that sum to 1. Meters behind
 *      the shooter or beyond the target contribute 0 (they can't see the shot).
 *
 * POSITION SOURCE IS DELIBERATELY OUT OF SCOPE HERE.
 *   This module takes positions as input. Whether a meter's position comes from
 *   its GPS (already in every NodePacket) or from a manual field in the WebUI is
 *   a wiring decision made by the caller -- both feed the same functions below.
 *
 * COST / CACHING
 *   update() recomputes the curve only when a ballistic input that affects it
 *   changes (MV/BC/model/weight/cal/twist/blen/range/zero/sight-ht/temp/pres).
 *   Wind speed/direction/angle do NOT change the curve, so a moving wind field
 *   is free. The recompute itself is heavy (nSeg+2 double-precision integrations)
 *   -- guarded here so it is safe to call every UI poll, but it must never be
 *   forced on the wind cadence. Weighting meters (weightsFor*) is cheap and can
 *   run every poll.
 */

#include <math.h>
#include <stdint.h>
#include "Ballistics.h"

#ifndef SENS_MAX_SEG
#define SENS_MAX_SEG 64
#endif

class Sensitivity {
private:
    Ballistics solver;
    float coef[SENS_MAX_SEG];       // inches / mph, per segment (magnitude)
    int   nSeg  = 0;
    float segYd = 0.0f;
    bool  valid = false;

    // cached ballistic inputs (for change detection)
    int   k_model = -1;
    float k_mv=0,k_bc=0,k_wgt=0,k_cal=0,k_twist=0,k_blen=0,
          k_range=0,k_zero=0,k_sh=0,k_tempF=0,k_pres=0;

    static bool diff(float a, float b) { return fabsf(a - b) > 1e-4f * (1.0f + fabsf(b)); }

public:
    // Recompute the sensitivity curve iff a curve-affecting input changed.
    // Returns true if it actually recomputed (heavy), false on a cache hit.
    bool update(const BallInput& in) {
        bool dirty = !valid
            || k_model != in.dragModel
            || diff(k_mv, in.mv_fps)    || diff(k_bc, in.bc)
            || diff(k_wgt, in.weight_gr)|| diff(k_cal, in.cal_in)
            || diff(k_twist, in.twist_in) || diff(k_blen, in.blen_in)
            || diff(k_range, in.range_yd) || diff(k_zero, in.zero_yd)
            || diff(k_sh, in.sightHt_in)  || diff(k_tempF, in.tempF)
            || diff(k_pres, in.presInHg);
        if (!dirty) return false;

        nSeg  = solver.windSensitivity(in, coef, SENS_MAX_SEG, &segYd);
        valid = (nSeg > 0);
        if (valid) {
            k_model=in.dragModel; k_mv=in.mv_fps; k_bc=in.bc; k_wgt=in.weight_gr;
            k_cal=in.cal_in; k_twist=in.twist_in; k_blen=in.blen_in; k_range=in.range_yd;
            k_zero=in.zero_yd; k_sh=in.sightHt_in; k_tempF=in.tempF; k_pres=in.presInHg;
        }
        return true;
    }

    bool  ready()      const { return valid; }
    int   segments()   const { return nSeg; }
    float segWidthYd() const { return segYd; }
    float rangeYd()    const { return nSeg * segYd; }

    // Raw sensitivity (inches/mph) at a downrange distance, linearly interpolated
    // between segment centers. Behind the muzzle or beyond the target -> 0.
    float sensitivityAt(float downrangeYd) const {
        if (!valid || downrangeYd < 0.0f) return 0.0f;
        float maxYd = nSeg * segYd;
        if (downrangeYd > maxYd) return 0.0f;
        float pos = downrangeYd / segYd - 0.5f;      // in "segment center" units
        if (pos <= 0.0f)          return coef[0];
        if (pos >= nSeg - 1)      return coef[nSeg - 1];
        int   i = (int)pos;
        float f = pos - (float)i;
        return coef[i] * (1.0f - f) + coef[i + 1] * f;
    }

    // Project a meter's GPS onto the firing line (base position + azimuth).
    // Returns signed downrange yards (negative = behind the shooter).
    // Local equirectangular ENU approximation -- accurate well past any range a
    // rifle shot cares about.
    static float projectDownrangeYd(double baseLat, double baseLon, float azimuthDeg,
                                    double meterLat, double meterLon,
                                    float* crossYdOut = nullptr) {
        const double M_PER_DEG = 111319.4908;   // meters per degree of latitude
        const double M2YD      = 1.0936133;
        double lat0 = baseLat * M_PI / 180.0;
        double dN = (meterLat - baseLat) * M_PER_DEG * M2YD;              // north, yd
        double dE = (meterLon - baseLon) * M_PER_DEG * cos(lat0) * M2YD;  // east,  yd
        double A  = azimuthDeg * M_PI / 180.0;   // 0 = north, 90 = east
        double downrange =  dN * cos(A) + dE * sin(A);
        double cross     = -dN * sin(A) + dE * cos(A);
        if (crossYdOut) *crossYdOut = (float)cross;
        return (float)downrange;
    }

    // ---- meter weighting -----------------------------------------------------
    // Fill outW[0..n-1] with normalized weights (sum to 1) from per-meter
    // downrange distances. Meters with 0 sensitivity (out of the flight path)
    // get weight 0. If NO meter is on the path, everything is 0 (caller decides
    // the fallback, e.g. equal weights).
    void weightsForDownranges(const float* downrangeYd, int n, float* outW) const {
        float sum = 0.0f;
        for (int i = 0; i < n; i++) { outW[i] = sensitivityAt(downrangeYd[i]); sum += outW[i]; }
        if (sum > 0.0f) for (int i = 0; i < n; i++) outW[i] /= sum;
    }

    // Same, but from meter GPS positions + the firing line.
    void weightsForMeterGps(double baseLat, double baseLon, float azimuthDeg,
                            const double* meterLat, const double* meterLon, int n,
                            float* outW) const {
        float sum = 0.0f;
        for (int i = 0; i < n; i++) {
            float d = projectDownrangeYd(baseLat, baseLon, azimuthDeg,
                                         meterLat[i], meterLon[i]);
            outW[i] = sensitivityAt(d);
            sum += outW[i];
        }
        if (sum > 0.0f) for (int i = 0; i < n; i++) outW[i] /= sum;
    }
};

#endif // SENSITIVITY_H
