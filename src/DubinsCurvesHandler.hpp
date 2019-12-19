#ifndef DUBINSCURVES
#define DUBINSCURVES

#include <cmath>
#include <vector>
#include <cstdint>

//region Structures

struct ScaledParameters {
    double scaled_th0, scaled_thf, scaled_k_max, inverse_scaled_k_max, lambda;
};

struct ScaledCurveSegments {
    double s1, s2, s3;
};

struct DubinsLine {
    double xf, yf, thf;
};

struct DubinsArc {
    double k, L, xf, yf, thf;
};

struct DubinsCurve {
    DubinsArc arcs[3];
    double L;
};
//endregion Structures

class DubinsCurvesHandler {
    private:
        //region Predefined dubins curves sets

        enum possible_curves {
            LSL, RSR, LSR, RSL, RLR, LRL, AMOUNT_OF_POSSIBLE_CURVES
        };

        int8_t possible_curves_corresponding_arguments[6][3] = {
                {1, 0, 1},
                {-1, 0, -1},
                {1, 0, -1},
                {-1, 0, 1},
                {-1, 1, -1},
                {1, -1, 1}
        };
        //endregion Predefined dubins curves sets

        double k_max = 3.0;

        DubinsLine calculateDubinsLine(double L, double x0, double y0, double th0, double k);
        DubinsArc calculateDubinsArc(double x0, double y0, double th0, double k, double L);
        DubinsCurve calculateDubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);
        bool isValid(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

        // Helpers
        double sinc(double t);
        double mod2pi(double angle);
        double normAng(double angle);
        ScaledParameters scaleToStandardForm(double x0, double y0, double th0, double xf, double yf, double thf);
        ScaledCurveSegments scaleFromStandard(ScaledCurveSegments scaled_curve_segments, double lambda);
        ScaledCurveSegments useLSL(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max);
        ScaledCurveSegments useRSR(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max);
        ScaledCurveSegments useLSR(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max);
        ScaledCurveSegments useRSL(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max);
        ScaledCurveSegments useRLR(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max);
        ScaledCurveSegments useLRL(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max);

    public:
        DubinsCurvesHandler() = default;
        explicit DubinsCurvesHandler(double k_max);
        DubinsCurve findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf);

};

#endif
