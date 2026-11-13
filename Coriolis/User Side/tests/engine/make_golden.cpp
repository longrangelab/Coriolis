// Host build of the ESP32 solver (lib/Ballistics/Ballistics.h) that writes
// golden_ballistics_h.json: inputs + outputs for a grid of shots, so the phone
// engine (lib/WebUI/EngineJs.h) can be checked against the firmware solver.
//
//   g++ -O2 -I../../lib/Ballistics make_golden.cpp -o make_golden && ./make_golden > golden_ballistics_h.json
#include <cstdio>
#include <initializer_list>
#include "Ballistics.h"

struct B { const char* name; float mv, bc; int model; float wgt, cal, twist, blen; };

int main() {
    const B bullets[] = {
        {"6.5CM 140 G7 .315", 2710, 0.315f, 7, 140, 0.264f, 8.0f, 1.41f},
        {".308 175 G7 .243",  2650, 0.243f, 7, 175, 0.308f, 10.0f, 1.24f},
        {".308 175 G1 .505",  2650, 0.505f, 1, 175, 0.308f, 10.0f, 1.24f},
        {".338 300 G7 .419",  2750, 0.419f, 7, 300, 0.338f, 9.4f, 1.70f},
        {".223 77 G7 .186",   2750, 0.186f, 7, 77,  0.224f, 7.0f, 0.99f},
    };
    const float ranges[] = {200, 500, 800, 1000, 1400};
    const float atm[][2] = {{59, 29.92f}, {20, 25.5f}, {95, 24.0f}};
    const float winds[][2] = {{10, 90}, {7, 300}, {15, 45}};
    const int twistDirs[] = {1, -1};
    Ballistics s;
    printf("{\"solve\":[\n");
    bool first = true;
    for (const B& b : bullets) for (float r : ranges) for (int a = 0; a < 3; a++) for (int w = 0; w < 3; w++)
    for (int td : twistDirs) for (int earth = 0; earth < 2; earth++) {
        if (earth && (w != 0 || a != 0)) continue;     // a few earth cases are enough
        if (td < 0 && w != 1) continue;
        BallInput in{};
        in.mv_fps = b.mv; in.bc = b.bc; in.dragModel = b.model; in.weight_gr = b.wgt; in.cal_in = b.cal;
        in.twist_in = b.twist; in.blen_in = b.blen; in.twistDir = td; in.sightHt_in = 1.75f; in.zero_yd = 100;
        in.range_yd = r; in.tempF = atm[a][0]; in.presInHg = atm[a][1]; in.wind_mph = winds[w][0];
        in.windRel_deg = winds[w][1]; in.lat_deg = 45; in.azimuth_deg = 90;
        in.useSpinDrift = true; in.useAeroJump = true; in.useEarth = earth;
        BallOutput o = s.solve(in);
        printf("%s{\"name\":\"%s\",\"in\":{\"mv\":%g,\"bc\":%g,\"model\":%d,\"wgt\":%g,\"cal\":%g,\"twist\":%g,\"blen\":%g,"
               "\"twistDir\":%d,\"sh\":1.75,\"zero\":100,\"range\":%g,\"tempF\":%g,\"pres\":%g,\"windmph\":%g,\"windrel\":%g,"
               "\"lat\":45,\"az\":90,\"earth\":%d},\"out\":{\"ok\":%s,\"elevMil\":%.5f,\"windMil\":%.5f,\"spinDriftIn\":%.4f,"
               "\"aeroJumpMOA\":%.5f,\"tof\":%.5f,\"vRemain\":%.2f,\"sg\":%.4f}}",
               first ? "" : ",\n", b.name, b.mv, b.bc, b.model, b.wgt, b.cal, b.twist, b.blen, td, r, in.tempF,
               in.presInHg, in.wind_mph, in.windRel_deg, earth, o.ok ? "true" : "false", o.elevMil, o.windMil,
               o.spinDriftIn, o.aeroJumpMOA, o.tof, o.vRemain_fps, o.sg);
        first = false;
    }
    // wind sensitivity curves
    printf("\n],\n\"sensitivity\":[\n");
    first = true;
    for (const B& b : bullets) for (float r : {600.0f, 1000.0f}) {
        BallInput in{};
        in.mv_fps = b.mv; in.bc = b.bc; in.dragModel = b.model; in.weight_gr = b.wgt; in.cal_in = b.cal;
        in.twist_in = b.twist; in.blen_in = b.blen; in.twistDir = 1; in.sightHt_in = 1.75f; in.zero_yd = 100;
        in.range_yd = r; in.tempF = 59; in.presInHg = 29.92f;
        float coef[64]; float segYd = 0;
        int n = s.windSensitivity(in, coef, 64, &segYd);
        printf("%s{\"name\":\"%s\",\"range\":%g,\"segYd\":%.4f,\"coef\":[", first ? "" : ",\n", b.name, r, segYd);
        for (int i = 0; i < n; i++) printf("%s%.5f", i ? "," : "", coef[i]);
        printf("]}");
        first = false;
    }
    printf("\n]}\n");
}
