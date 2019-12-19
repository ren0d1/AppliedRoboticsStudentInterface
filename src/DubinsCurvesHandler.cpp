#include "DubinsCurvesHandler.hpp"

#include <iostream>

/* Constructors */
DubinsCurvesHandler::DubinsCurvesHandler(double k_max){
    this->k_max = k_max;
}

//region Inner Logic functions (private)

DubinsLine DubinsCurvesHandler::calculateDubinsLine(double s, double x0, double y0, double th0, double k){
    DubinsLine dubins_line{};

    dubins_line.xf = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
    dubins_line.yf = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
    dubins_line.thf = mod2pi(th0 + k * s);

    return dubins_line;
}

DubinsArc DubinsCurvesHandler::calculateDubinsArc(double x0, double y0, double th0, double k, double L){
    DubinsArc dubins_arc{};

    dubins_arc.k = k;
    dubins_arc.L = L;

    DubinsLine corresponding_dubins_line = calculateDubinsLine(L, x0, y0, th0, k);
    dubins_arc.xf = corresponding_dubins_line.xf;
    dubins_arc.yf = corresponding_dubins_line.yf;
    dubins_arc.thf = corresponding_dubins_line.thf;

    return dubins_arc;
}

DubinsCurve DubinsCurvesHandler::calculateDubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2){
    DubinsCurve dubins_curve{};

    dubins_curve.arcs[0] = calculateDubinsArc(x0, y0, th0, k0, s1);
    dubins_curve.arcs[1] = calculateDubinsArc(dubins_curve.arcs[0].xf, dubins_curve.arcs[0].yf, dubins_curve.arcs[0].thf, k1, s2);
    dubins_curve.arcs[2] = calculateDubinsArc(dubins_curve.arcs[1].xf, dubins_curve.arcs[1].yf, dubins_curve.arcs[1].thf, k2, s3);

    dubins_curve.L = dubins_curve.arcs[0].L + dubins_curve.arcs[1].L + dubins_curve.arcs[2].L;

    return dubins_curve;
}

bool DubinsCurvesHandler::isValid(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf){
    double x0 = -1;
    double y0 = 0;
    double xf = 1;
    double yf = 0;

    double eq1 = x0 + s1 * sinc(0.5 * k0 * s1) * cos(th0 + 0.5 * k0 * s1) + s2 * sinc(0.5 * k1 * s2) * cos(th0 + k0 * s1 + 0.5 * k1 * s2) + s3 * sinc(0.5 * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + 0.5 * k2 * s3) - xf;
    std::cout << "eq1: " << std::to_string(eq1) << std::endl;
    double eq2 = y0 + s1 * sinc(0.5 * k0 * s1) * sin(th0 + 0.5 * k0 * s1) + s2 * sinc(0.5 * k1 *s2) * sin(th0 + k0 * s1 + 0.5 * k1 * s2) + s3 * sinc(0.5 * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + 0.5 * k2 * s3) - yf;
    std::cout << "eq2: " << std::to_string(eq2) << std::endl;
    double eq3 = normAng(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);
    std::cout << "eq3: " << std::to_string(eq3) << std::endl;

    bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
    return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < pow(10, -10)) && Lpos;
}
//endregion Inner Logic functions (private)

//region Helpers

double DubinsCurvesHandler::sinc(double t){
    double answer;
    if(std::abs(t) < 0.002)
    {
        answer = 1 - pow(t, 2) / 6 * (1 - pow(t, 2) / 20);
    }else{
        answer = sin(t) / t;
    }

    return answer;
}

double DubinsCurvesHandler::mod2pi(double angle) {
    double newAngle = angle;
    while (newAngle < 0)
    {
        newAngle = newAngle + 2 * M_PI;
    }

    while(newAngle >= 2 * M_PI)
    {
        newAngle = newAngle - 2 * M_PI;
    }

    return newAngle;
}

double DubinsCurvesHandler::normAng(double angle) {
    double newAngle = angle;
    while(newAngle <= -M_PI)
    {
        newAngle = newAngle + 2 * M_PI;
    }

    while(newAngle >= 2 * M_PI)
    {
        newAngle = newAngle - 2 * M_PI;
    }

    return newAngle;
}

ScaledParameters DubinsCurvesHandler::scaleToStandardForm(double x0, double y0, double th0, double xf, double yf, double thf){
    ScaledParameters scaled_parameters;

    // Finding transform parameters
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    scaled_parameters.lambda = hypot(dx, dy) / 2;

    // Scaling and normalizing angles and curvatures
    scaled_parameters.scaled_th0 = mod2pi(th0 - phi);
    scaled_parameters.scaled_thf = mod2pi(thf - phi);
    scaled_parameters.scaled_k_max = k_max * scaled_parameters.lambda;
    scaled_parameters.inverse_scaled_k_max = 1 / scaled_parameters.scaled_k_max;

    return scaled_parameters;
}

ScaledCurveSegments DubinsCurvesHandler::scaleFromStandard(ScaledCurveSegments scaled_curve_segments, double lambda) {
    scaled_curve_segments.s1 = scaled_curve_segments.s1 * lambda;
    scaled_curve_segments.s2 = scaled_curve_segments.s2 * lambda;
    scaled_curve_segments.s3 = scaled_curve_segments.s3 * lambda;

    return scaled_curve_segments;
}

ScaledCurveSegments DubinsCurvesHandler::useLSL(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max){
    double C = cos(scaled_thf) - cos(scaled_th0);
    double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);

    std::cout << "LSL C: " << std::to_string(C) << std::endl;
    std::cout << "LSL S: " << std::to_string(S) << std::endl;

    ScaledCurveSegments scaled_curve_segments{};

    double temp = 2.0 + 4 * pow(scaled_k_max, 2) - 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf));
    std::cout << "LSL Temp2: " << std::to_string(temp) << std::endl;
    if (temp >= 0)
    {
        scaled_curve_segments.s2 = inverse_scaled_k_max * sqrt(temp);

        temp = atan2(C, S);
        scaled_curve_segments.s1 = inverse_scaled_k_max * mod2pi(temp - scaled_th0);
        scaled_curve_segments.s3 = inverse_scaled_k_max * mod2pi(scaled_thf - temp);

        std::cout << "LSL Temp: " << std::to_string(temp) << std::endl;
        std::cout << "LSL s1: " << std::to_string(scaled_curve_segments.s1) << std::endl;
        std::cout << "LSL s2: " << std::to_string(scaled_curve_segments.s2) << std::endl;
        std::cout << "LSL s3: " << std::to_string(scaled_curve_segments.s3) << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;

    return scaled_curve_segments;
}

ScaledCurveSegments DubinsCurvesHandler::useRSR(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max){
    double C = cos(scaled_th0) - cos(scaled_thf);
    double S = 2 * scaled_k_max - sin(scaled_th0) + sin(scaled_thf);
    std::cout << "RSR C: " << std::to_string(C) << std::endl;
    std::cout << "RSR S: " << std::to_string(S) << std::endl;

    ScaledCurveSegments scaled_curve_segments{};

    double temp = 2 + 4 * pow(scaled_k_max, 2) - 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf));
    std::cout << "RSR Temp2: " << std::to_string(temp) << std::endl;
    if (temp >= 0)
    {
        scaled_curve_segments.s2 = inverse_scaled_k_max * sqrt(temp);

        temp = atan2(C, S);
        scaled_curve_segments.s1 = inverse_scaled_k_max * mod2pi(scaled_th0 - temp);
        scaled_curve_segments.s3 = inverse_scaled_k_max * mod2pi(temp - scaled_thf);

        std::cout << "RSR Temp: " << std::to_string(temp) << std::endl;
        std::cout << "RSR s1: " << std::to_string(scaled_curve_segments.s1) << std::endl;
        std::cout << "RSR s2: " << std::to_string(scaled_curve_segments.s2) << std::endl;
        std::cout << "RSR s3: " << std::to_string(scaled_curve_segments.s3) << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;

    return scaled_curve_segments;
}

ScaledCurveSegments DubinsCurvesHandler::useLSR(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max){
    double C = cos(scaled_th0) + cos(scaled_thf);
    double S = 2 * scaled_k_max + sin(scaled_th0) + sin(scaled_thf);
    std::cout << "LSR C: " << std::to_string(C) << std::endl;
    std::cout << "LSR S: " << std::to_string(S) << std::endl;

    ScaledCurveSegments scaled_curve_segments{};

    double temp = 4 * pow(scaled_k_max, 2) - 2 + 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) + sin(scaled_thf));
    std::cout << "LSR Temp3: " << std::to_string(temp) << std::endl;
    if (temp >= 0)
    {
        scaled_curve_segments.s2 = inverse_scaled_k_max * sqrt(temp);

        temp = atan2(-C, S);
        double temp2 = -atan2(-2, scaled_curve_segments.s2 * scaled_k_max);
        scaled_curve_segments.s1 = inverse_scaled_k_max * mod2pi(temp + temp2 - scaled_th0);
        scaled_curve_segments.s3 = inverse_scaled_k_max * mod2pi(temp + temp2 - scaled_thf);

        std::cout << "LSR Temp: " << std::to_string(temp) << std::endl;
        std::cout << "LSR Temp2: " << std::to_string(temp2) << std::endl;
        std::cout << "LSR s1: " << std::to_string(scaled_curve_segments.s1) << std::endl;
        std::cout << "LSR s2: " << std::to_string(scaled_curve_segments.s2) << std::endl;
        std::cout << "LSR s3: " << std::to_string(scaled_curve_segments.s3) << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;

    return scaled_curve_segments;
}

ScaledCurveSegments DubinsCurvesHandler::useRSL(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max){
    double C = cos(scaled_th0) + cos(scaled_thf);
    double S = 2 * scaled_k_max - sin(scaled_th0) - sin(scaled_thf);
    std::cout << "RSL C: " << std::to_string(C) << std::endl;
    std::cout << "RSL S: " << std::to_string(S) << std::endl;

    ScaledCurveSegments scaled_curve_segments{};

    double temp = 4 * pow(scaled_k_max, 2) - 2 + 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) + sin(scaled_thf));
    std::cout << "RSL Temp3: " << std::to_string(temp) << std::endl;
    if (temp >= 0)
    {
        scaled_curve_segments.s2 = inverse_scaled_k_max * sqrt(temp);

        temp = atan2(C, S);
        double temp2 = atan2(2, scaled_curve_segments.s2 * scaled_k_max);
        scaled_curve_segments.s1 = inverse_scaled_k_max * mod2pi(scaled_th0 - temp + temp2);
        scaled_curve_segments.s3 = inverse_scaled_k_max * mod2pi(scaled_thf - temp + temp2);

        std::cout << "RSL Temp: " << std::to_string(temp) << std::endl;
        std::cout << "RSL Temp2: " << std::to_string(temp2) << std::endl;
        std::cout << "RSL s1: " << std::to_string(scaled_curve_segments.s1) << std::endl;
        std::cout << "RSL s2: " << std::to_string(scaled_curve_segments.s2) << std::endl;
        std::cout << "RSL s3: " << std::to_string(scaled_curve_segments.s3) << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;

    return scaled_curve_segments;
}

ScaledCurveSegments DubinsCurvesHandler::useRLR(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max){
    double C = cos(scaled_thf) - cos(scaled_th0);
    double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);
    std::cout << "RLR C: " << std::to_string(C) << std::endl;
    std::cout << "RLR S: " << std::to_string(S) << std::endl;

    ScaledCurveSegments scaled_curve_segments{};

    double temp = 0.125 * (6 - 4 * pow(scaled_k_max, 2) + 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf)));
    std::cout << "RLR Temp2: " << std::to_string(temp) << std::endl;
    if (temp >= 0)
    {
        scaled_curve_segments.s2 = inverse_scaled_k_max * mod2pi(2 * M_PI - acos(temp));
        temp = atan2(C, S);
        scaled_curve_segments.s1 = inverse_scaled_k_max * mod2pi(scaled_th0 - temp + 0.5 * scaled_curve_segments.s2 * scaled_k_max);
        scaled_curve_segments.s3 = inverse_scaled_k_max * mod2pi(scaled_th0 - scaled_thf + scaled_k_max * (scaled_curve_segments.s2 - scaled_curve_segments.s1));

        std::cout << "RLR Temp: " << std::to_string(temp) << std::endl;
        std::cout << "RLR s1: " << std::to_string(scaled_curve_segments.s1) << std::endl;
        std::cout << "RLR s2: " << std::to_string(scaled_curve_segments.s2) << std::endl;
        std::cout << "RLR s3: " << std::to_string(scaled_curve_segments.s3) << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;

    return scaled_curve_segments;
}

ScaledCurveSegments DubinsCurvesHandler::useLRL(double scaled_th0, double scaled_thf, double scaled_k_max, double inverse_scaled_k_max){
    double C = cos(scaled_thf) - cos(scaled_th0);
    double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);
    std::cout << "LRL C: " << std::to_string(C) << std::endl;
    std::cout << "LRL S: " << std::to_string(S) << std::endl;

    ScaledCurveSegments scaled_curve_segments{};

    double temp = 0.125 * (6 - 4 * pow(scaled_k_max, 2) + 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf)));
    std::cout << "LRL Temp2: " << std::to_string(temp) << std::endl;
    if (temp >= 0)
    {
        scaled_curve_segments.s2 = inverse_scaled_k_max * mod2pi(2 * M_PI - acos(temp));

        temp = atan2(C, S);
        scaled_curve_segments.s1 = inverse_scaled_k_max * mod2pi(temp - scaled_th0 + 0.5 * scaled_curve_segments.s2 * scaled_k_max);
        scaled_curve_segments.s3 = inverse_scaled_k_max * mod2pi(scaled_thf - scaled_th0 + scaled_k_max * (scaled_curve_segments.s2 - scaled_curve_segments.s1));

        std::cout << "LRL Temp: " << std::to_string(temp) << std::endl;
        std::cout << "LRL s1: " << std::to_string(scaled_curve_segments.s1) << std::endl;
        std::cout << "LRL s2: " << std::to_string(scaled_curve_segments.s2) << std::endl;
        std::cout << "LRL s3: " << std::to_string(scaled_curve_segments.s3) << std::endl;
    }

    std::cout << "------------------------------------------------------" << std::endl;

    return scaled_curve_segments;
}
//endregion Helpers

//region Utility functions (public)

DubinsCurve DubinsCurvesHandler::findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf) {
    ScaledParameters scaled_parameters = scaleToStandardForm(x0, y0, th0, xf, yf, thf);

    ScaledCurveSegments scaled_curved_segments{};

    ScaledCurveSegments best_scaled_curved_segments{};
    double L = INFINITY;
    int best_index = -1;
    for(int i = 0; i < AMOUNT_OF_POSSIBLE_CURVES; i++)
    {
        switch (i)
        {
            case LSL:
                scaled_curved_segments = useLSL(scaled_parameters.scaled_th0, scaled_parameters.scaled_thf, scaled_parameters.scaled_k_max, scaled_parameters.inverse_scaled_k_max);

                break;

            case RSR:
                scaled_curved_segments = useRSR(scaled_parameters.scaled_th0, scaled_parameters.scaled_thf, scaled_parameters.scaled_k_max, scaled_parameters.inverse_scaled_k_max);
                break;

            case LSR:
                scaled_curved_segments = useLSR(scaled_parameters.scaled_th0, scaled_parameters.scaled_thf, scaled_parameters.scaled_k_max, scaled_parameters.inverse_scaled_k_max);
                break;

            case RSL:
                scaled_curved_segments = useRSL(scaled_parameters.scaled_th0, scaled_parameters.scaled_thf, scaled_parameters.scaled_k_max, scaled_parameters.inverse_scaled_k_max);
                break;

            case RLR:
                scaled_curved_segments = useRLR(scaled_parameters.scaled_th0, scaled_parameters.scaled_thf, scaled_parameters.scaled_k_max, scaled_parameters.inverse_scaled_k_max);
                break;

            case LRL:
                scaled_curved_segments = useLRL(scaled_parameters.scaled_th0, scaled_parameters.scaled_thf, scaled_parameters.scaled_k_max, scaled_parameters.inverse_scaled_k_max);
                break;

            default:
                scaled_curved_segments.s1 = 0;
                scaled_curved_segments.s2 = 0;
                scaled_curved_segments.s3 = 0;
                break;
        }

        double current_L = scaled_curved_segments.s1 + scaled_curved_segments.s2 + scaled_curved_segments.s3;

        if((scaled_curved_segments.s1 != 0 || scaled_curved_segments.s2 != 0 || scaled_curved_segments.s3 != 0) && current_L < L)
        {
            L = current_L;
            best_index = i;
            best_scaled_curved_segments.s1 = scaled_curved_segments.s1;
            best_scaled_curved_segments.s2 = scaled_curved_segments.s2;
            best_scaled_curved_segments.s3 = scaled_curved_segments.s3;
        }
    }

    if (best_index != -1)
    {
        ScaledCurveSegments converted_back_scaled_curve_segments = scaleFromStandard(best_scaled_curved_segments, scaled_parameters.lambda);

        DubinsCurve curve = calculateDubinsCurve(x0, y0, th0, converted_back_scaled_curve_segments.s1, converted_back_scaled_curve_segments.s2, converted_back_scaled_curve_segments.s3, possible_curves_corresponding_arguments[best_index][0] * k_max, possible_curves_corresponding_arguments[best_index][1] * k_max, possible_curves_corresponding_arguments[best_index][2] * k_max);

        std::cout << "Dubins curve L: " << std::to_string(curve.L) << std::endl;
        std::cout << "Strategy: " << best_index << std::endl;
        std::cout << "------------------------------------------------------" << std::endl;

        for(int i = 0; i < 3; i++){
            std::cout << "Dubins curve L: " << std::to_string(curve.arcs[i].L) << std::endl;
            std::cout << "Dubins curve k: " << std::to_string(curve.arcs[i].k) << std::endl;
            std::cout << "Dubins curve x: " << std::to_string(curve.arcs[i].xf) << std::endl;
            std::cout << "Dubins curve y: " << std::to_string(curve.arcs[i].yf) << std::endl;
            std::cout << "Dubins curve th: " << std::to_string(curve.arcs[i].thf) << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;
        }

        bool valid = isValid(best_scaled_curved_segments.s1, possible_curves_corresponding_arguments[best_index][0] * scaled_parameters.scaled_k_max, best_scaled_curved_segments.s2, possible_curves_corresponding_arguments[best_index][1] * scaled_parameters.scaled_k_max, best_scaled_curved_segments.s3, possible_curves_corresponding_arguments[best_index][2] * scaled_parameters.scaled_k_max, scaled_parameters.scaled_th0, scaled_parameters.scaled_thf);
        if(valid)
            return curve;

        // Throw error if not valid
    }

    return {};
}
//endregion Utility functions (public)