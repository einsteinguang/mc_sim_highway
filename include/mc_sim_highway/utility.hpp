#ifndef SRC_UTILITY_HPP
#define SRC_UTILITY_HPP
#pragma once
#include "types.hpp"
#include "agent.hpp"
#include <optional>

namespace mc_sim_highway{

inline double timeToCollision(const double& d, const double& v1, const double& v2){
    if (v1 - v2 < 1e-10){
        return 1e10;
    }
    return d / (v1 - v2);
}

inline double reverseTimeToCollision(const double& d, const double& v1, const double& v2){
    if (d == 0){
        return (v1 - v2) / 1e-3;
    }
    return (v1 - v2) / d;
}

inline double noUnitDistanceChangingRate(const double& v1, const double& v2){
    if (v1 == 0){
        return (v1 - v2) / 1e-3;
    }
    return (v1 - v2) / v1;
}

inline double idmAccFrontBack(const IdmParam& p, const IdmMatches& fs, const IdmMatch& b, const double& egoV, const double& vd){
    const double accMax = p.accMax;
    const double minDis = p.minDis;
    const double accMin = p.accMin;
    const double accCom = p.accCom;
    const double thw = p.thw;
    const double freeRoadTerm = accMax * (1 - std::pow((egoV / vd), 4));
    double dTerm = 0;
    std::vector<double> dTermF;
    for(const auto& f: fs){
        if (f.id >= 0){
            if(f.dis > 0){
                dTermF.push_back(accMin *
                                   std::pow((std::max(0., (0.7 * (minDis + egoV * thw) +
                                                           egoV * (egoV - f.v) / (2 * std::sqrt(accMin * accCom)))) /
                                             f.dis), 2));
            } else{
                dTermF.push_back(-1e10);
            }
        }
    }
    if (!dTermF.empty()){
        dTerm = dTerm + *std::min_element(dTermF.begin(), dTermF.end());
    }
    if (b.id >= 0){
        if(b.dis < 0){
            dTerm = dTerm + accMax *
                               std::pow((std::max(0., (0.7 * (minDis + b.v * thw) +
                                                       egoV * (b.v - egoV) / (2 * std::sqrt(accMin * accCom)))) /
                                         -b.dis), 2);
        } else{
            dTerm = dTerm + 1e10;
        }
    }
    const double acc = freeRoadTerm + dTerm;
    return std::max(std::min(accMax, acc), accMin);
}

inline double idmAccFrontBack(const IdmParam& p, const AgentsPointer& fs, const AgentPointer& b,
                              const double& egoX, const double& egoV, const double& egoL, const double& vd){
    const double accMax = p.accMax;
    const double minDis = p.minDis;
    const double accMin = p.accMin;
    const double accCom = p.accCom;
    const double thw = p.thw;
    const double freeRoadTerm = accMax * (1 - std::pow((egoV / vd), 4));
    double dTerm = 0;
    std::vector<double> dTermF;
    for(const auto& f: fs){
        if (f){
            const double realDisF = f->x - egoX - 0.5 * (egoL + f->l);
            if(realDisF > 0){
                dTermF.push_back(-accMax *
                                 std::pow((std::max(0., (0.7 * (minDis + egoV * thw) +
                                                         egoV * (egoV - f->v) / (2 * std::sqrt(-accMax * accCom)))) /
                                           realDisF), 2));
            } else{
                dTermF.push_back(-1e10);
            }
        }
    }
    if (!dTermF.empty()){
        dTerm = dTerm + *std::min_element(dTermF.begin(), dTermF.end());
    }
    if (b){
        const double realDisB = b->x - egoX - 0.5 * (egoL + b->l);
        if(realDisB < 0){
            dTerm = dTerm + accMax *
                            std::pow((std::max(0., (0.7 * (minDis + b->v * thw) +
                                                    egoV * (b->v - egoV) / (2 * std::sqrt(-accMax * accCom)))) /
                                      -realDisB), 2);
        } else{
            dTerm = dTerm + 1e10;
        }
    }
    const double acc = freeRoadTerm + dTerm;
    return std::max(std::min(accMax, acc), accMin);
}

inline double rssDis(const double& vf, const double& vb, const double& rt, const double& aMin, const double& aMax){
    return std::max(vb * rt + vb * vb / (-2 * aMin) - vf * vf / (-2 * aMax), 0.);
}

inline double aNeed(const double& vf, const double&vb, const double& rt, const double& aMax, const double& dis){
    const double a = -0.5 * vb * vb / std::max(0.01, dis + vf * vf / (-2 * aMax) - vb * rt);
    return std::min(std::max(a, aMax), 0.5 * aMax);
}

bool rssSafe(const IdmMatches& fs, const IdmMatch& b,
             const double& v0, const double vd, const double& backVehicleSoftDcc,
             const RSSParam& rssP, const IdmParam& idmP){
    // fs: [---ego---fs[0]---fs[1]---...]
    const double dt = 0.4;
    const int steps = 10;
    double egoV = v0;
    IdmMatch fCopy{-1, 0, 0};
    IdmMatch bCopy = b;
    if(!fs.empty()){ fCopy = fs[0]; }
    for(int i = 0; i < steps; i++){
        if(fCopy.id >= 0){
            const double rssDisF = rssDis(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMinBrake, rssP.aMaxBrake);
            if(fCopy.dis < rssDisF){ return false; }
        }
        if(bCopy.id >= 0){
            const double rssDisB = rssDis(egoV, bCopy.v, rssP.rTimeOther, rssP.aMinBrake, rssP.aMinBrake);
            if(-bCopy.dis < rssDisB){ return false; }
        }
        const double idmAccEgo = idmAccFrontBack(idmP, {fCopy}, bCopy, egoV, vd);
        const double egoDisplacement = std::max(0., egoV * dt + 0.5 * idmAccEgo * dt * dt);
        egoV = std::max(egoV + idmAccEgo * dt, 0.);
        if(fCopy.id >= 0){
            fCopy.dis = fCopy.dis + fCopy.v * dt - egoDisplacement;
        }
        if(bCopy.id >= 0){
            bCopy.dis = bCopy.dis + bCopy.v * dt - egoDisplacement;
            bCopy.v = std::max(0., bCopy.v - backVehicleSoftDcc * dt);
        }
    }
    return true;
}

bool rssSafeRelax(const IdmMatches& fs, const IdmMatch& b,
                  const double& v0, const double vd,
                  const RSSParam& rssP, const IdmParam& idmP){
    // fs: [---ego---fs[0]---fs[1]---...]
    const double dt = 0.5;
    const int steps = 8;
    double egoV = v0;
    IdmMatch fCopy{-1, 0, 0};
    IdmMatch ffCopy{-1, 0, 0};
    IdmMatch bCopy = b;
    if(!fs.empty()){ fCopy = fs[0]; }
    if(fs.size() > 1){ ffCopy = fs[1]; }
    for(int i = 0; i < steps; i++){
        double aNeedF = 0.5 * rssP.aMaxBrake;
        double aNeedEgo = 0.5 * rssP.aMaxBrake;
        if(ffCopy.id > 0){
            aNeedF = aNeed(ffCopy.v, fCopy.v, rssP.rTimeOther, rssP.aMaxBrake, ffCopy.dis - fCopy.dis);
        }
        if(fCopy.id >= 0){
            const double rssDisF = rssDis(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMinBrake, aNeedF);
            aNeedEgo = aNeed(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMaxBrake, fCopy.dis);
            if(fCopy.dis < rssDisF){ return false; }
        }
//        std::cout << "aNeedF: " << aNeedF << std::endl;
//        std::cout << "aNeedEgo: " << aNeedEgo << std::endl;
        if(bCopy.id >= 0){
            const double rssDisB = rssDis(egoV, bCopy.v, rssP.rTimeOther, rssP.aMinBrake, aNeedEgo);
            if(-bCopy.dis < rssDisB){ return false; }
        }
        const double idmAccEgo = idmAccFrontBack(idmP, {fCopy}, bCopy, egoV, vd);
        const double egoDisplacement = std::max(0., egoV * dt + 0.5 * idmAccEgo * dt * dt);
        egoV = std::max(egoV + idmAccEgo * dt, 0.);
        if(ffCopy.id >= 0){
            ffCopy.dis = ffCopy.dis + ffCopy.v * dt - egoDisplacement;
        }
        if(fCopy.id >= 0){
            fCopy.dis = fCopy.dis + fCopy.v * dt - egoDisplacement;
        }
        if(bCopy.id >= 0){
            bCopy.dis = bCopy.dis + bCopy.v * dt - 0.5 * rssP.d * dt * dt - egoDisplacement;
            bCopy.v = std::max(0., bCopy.v - rssP.d * dt);
        }
    }
    return true;
}


bool rssSafeRelax(const std::optional<double>& ffd,
                  const std::optional<double>& ffv,
                  const std::optional<double>& fd,
                  const std::optional<double>& fv,
                  const std::optional<double>& bd,
                  const std::optional<double>& bv,
                  const double& v0, const double vd,
                  const RSSParam& rssP, const IdmParam& idmP){
    // fs: [---ego---fs[0]---fs[1]---...]
    const double dt = 1.0;
    const int steps = 2;
    double egoV = v0;
    IdmMatch fCopy{-1, 0, 0};
    IdmMatch ffCopy{-1, 0, 0};
    IdmMatch bCopy{-1, 0, 0};
    if(bd){ bCopy = IdmMatch{1, *bd, *bv}; }
    if(fd){ fCopy = IdmMatch{1, *fd, *fv}; }
    if(ffd){ ffCopy = IdmMatch{1, *ffd, *ffv}; }
    for(int i = 0; i < steps; i++){
        double aNeedF = 0.5 * rssP.aMaxBrake;
        double aNeedEgo = 0.5 * rssP.aMaxBrake;
        if(ffCopy.id > 0){
            aNeedF = std::min(aNeed(ffCopy.v, fCopy.v, rssP.rTimeOther, rssP.aMaxBrake, ffCopy.dis - fCopy.dis), aNeedF);
        }
        if(fCopy.id >= 0){
            const double rssDisF = rssDis(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMinBrake, aNeedF);
            aNeedEgo = std::min(aNeed(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMaxBrake, fCopy.dis), aNeedEgo);
            if(fCopy.dis < rssDisF){ return false; }
        }
        if(bCopy.id >= 0){
            const double rssDisB = rssDis(egoV, bCopy.v, rssP.rTimeOther, rssP.aMinBrake, aNeedEgo);
            if(-bCopy.dis < rssDisB){ return false; }
        }
        const double idmAccEgo = idmAccFrontBack(idmP, {fCopy}, bCopy, egoV, vd);
        const double egoDisplacement = std::max(0., egoV * dt + 0.5 * idmAccEgo * dt * dt);
        egoV = std::max(egoV + idmAccEgo * dt, 0.);
        if(ffCopy.id >= 0){
            ffCopy.dis = ffCopy.dis + ffCopy.v * dt - egoDisplacement;
        }
        if(fCopy.id >= 0){
            fCopy.dis = fCopy.dis + fCopy.v * dt - egoDisplacement;
        }
        if(bCopy.id >= 0){
            bCopy.dis = bCopy.dis + bCopy.v * dt - 0.5 * rssP.d * dt * dt - egoDisplacement;
            bCopy.v = std::max(0., bCopy.v - rssP.d * dt);
        }
    }
    return true;
}

bool rssSafe(const std::optional<double>& fd,
             const std::optional<double>& fv,
             const std::optional<double>& bd,
             const std::optional<double>& bv,
             const double& v0, const double vd,
             const RSSParam& rssP, const IdmParam& idmP){
    // fs: [---ego---fs[0]---fs[1]---...]
    const double dt = 1.0;
    const int steps = 1;
    double egoV = v0;
    IdmMatch fCopy{-1, 0, 0};
    IdmMatch bCopy{-1, 0, 0};
    if(bd){ bCopy = IdmMatch{1, *bd, *bv}; }
    if(fd){ fCopy = IdmMatch{1, *fd, *fv}; }
    for(int i = 0; i < steps; i++){
        if(fCopy.id >= 0){
            const double rssDisF = rssDis(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMinBrake, rssP.aMaxBrake);
            if(fCopy.dis < rssDisF){ return false; }
        }
        if(bCopy.id >= 0){
            const double rssDisB = rssDis(egoV, bCopy.v, rssP.rTimeOther, rssP.aMinBrake, rssP.aMaxBrake);
            if(-bCopy.dis < rssDisB){ return false; }
        }
        const double idmAccEgo = idmAccFrontBack(idmP, {fCopy}, bCopy, egoV, vd);
        const double egoDisplacement = std::max(0., egoV * dt + 0.5 * idmAccEgo * dt * dt);
        egoV = std::max(egoV + idmAccEgo * dt, 0.);
        if(fCopy.id >= 0){
            fCopy.dis = fCopy.dis + fCopy.v * dt - egoDisplacement;
        }
        if(bCopy.id >= 0){
            bCopy.dis = bCopy.dis + bCopy.v * dt - 0.5 * rssP.d * dt * dt - egoDisplacement;
            bCopy.v = std::max(0., bCopy.v - rssP.d * dt);
        }
    }
    return true;
}

bool rssSafe(const std::optional<double>& fd,
             const std::optional<double>& fv,
             const std::optional<double>& bd,
             const std::optional<double>& bv,
             const double& v0,
             const RSSParam& rssP){
    double egoV = v0;
    IdmMatch fCopy{-1, 0, 0};
    IdmMatch bCopy{-1, 0, 0};
    if(bd){ bCopy = IdmMatch{1, *bd, *bv}; }
    if(fd){ fCopy = IdmMatch{1, *fd, *fv}; }
    if(fCopy.id >= 0){
        const double rssDisF = rssDis(fCopy.v, egoV, rssP.rTimeEgo, rssP.aMinBrake, rssP.aMaxBrake);
        if(fCopy.dis < rssDisF){ return false; }
    }
    if(bCopy.id >= 0){
        const double rssDisB = rssDis(egoV, bCopy.v, rssP.rTimeOther, rssP.aMinBrake, rssP.aMaxBrake);
        if(-bCopy.dis < rssDisB){ return false; }
    }
    return true;
}

inline double timeToReachGap(const double& sBackNow, const double& vBack,
                             const double& sFrontNow, const double& vFront,
                             const double& sEgo, const double& vEgo){
    double s;
    double v;
    if(sEgo < sBackNow){
        s = sBackNow;
        v = vBack;
    }else if(sEgo > sFrontNow){
        s = sFrontNow;
        v = vFront;
    }else{
        s = sEgo;
        v = 0.5 * vBack + 0.5 * vFront;
    }
    if(s < sEgo){
        const double timeToGap1 = 0.5 * (vEgo - v + std::sqrt(std::pow(v - vEgo, 2) - 4 * (s - sEgo)));
        const double timeToGap2 = 0.5 * (vEgo - v - std::sqrt(std::pow(v - vEgo, 2) - 4 * (s - sEgo)));
        return timeToGap1 > timeToGap2 ? timeToGap1 : timeToGap2;
    } else if (s > sEgo){
        const double timeToGap1 = 0.5 * (v - vEgo + std::sqrt(std::pow(v - vEgo, 2) + 4 * (s - sEgo)));
        const double timeToGap2 = 0.5 * (v - vEgo - std::sqrt(std::pow(v - vEgo, 2) + 4 * (s - sEgo)));
        return timeToGap1 > timeToGap2 ? timeToGap1 : timeToGap2;
    }else{
        return std::abs(v - vEgo);
    }
}

inline double timeToReachGap(const double& sBackNow, const double& vBack,
                             const double& sFrontNow, const double& vFront,
                             const double& sEgo, const double& vEgo, const double& vLimit){
    double s;
    double v;
    if(sEgo < sBackNow){
        s = sBackNow;
        v = vBack;
    }else if(sEgo > sFrontNow){
        s = sFrontNow;
        v = vFront;
    }else{
        s = sEgo;
        v = std::abs(vFront - vEgo) > std::abs(vBack - vEgo) ? vBack : vFront;
    }
    const double acc = std::max(std::min((vLimit - vEgo) / 5., 2.), 0.5);
    const double dcc = 2;
    if(s < sEgo){
        const double timeToGap1 = 1 / dcc * (vEgo - v + std::sqrt(std::pow(v - vEgo, 2) - 2 * dcc * (s - sEgo)));
        const double timeToGap2 = 1 / dcc * (vEgo - v - std::sqrt(std::pow(v - vEgo, 2) - 2 * dcc * (s - sEgo)));
        return timeToGap1 > timeToGap2 ? timeToGap1 : timeToGap2;
    } else if (s > sEgo){
        const double timeToGap1 = 1 / acc * (v - vEgo + std::sqrt(std::pow(v - vEgo, 2) + 2 * acc * (s - sEgo)));
        const double timeToGap2 = 1 / acc * (v - vEgo - std::sqrt(std::pow(v - vEgo, 2) + 2 * acc * (s - sEgo)));
        return timeToGap1 > timeToGap2 ? timeToGap1 : timeToGap2;
    }else{
        if(0.5 * v < vEgo){
            return 0.;
        }else{
            return 2 * (0.5 * v - vEgo) / acc;
        }
    }
}

inline LaneChangeProbability truckLaneChangeProbability(const LaneChangeProbability& estimate,
                                                        const std::optional<int>& leftLaneId,
                                                        const int& noTruckLaneId){
    // truck has way less lane change probability
    double leftQ = 0.5 * estimate.left;
    if(leftLaneId == noTruckLaneId){
        leftQ = 0.;
    }
    const double sum = leftQ + 2. * estimate.keep + 2. * estimate.right;
    return {leftQ, 2. * estimate.keep / sum, 2. * estimate.right / sum};
}

inline double weightsToLCInfoSource(const LaneChangeProbability& pFromOneSource){
    // max lc p == 0 -> 0.5 weight, max lc p == 1.0 -> 2 weight
    return 1.5 * pFromOneSource.maxLCP() + 0.5;
}

inline LaneChangeProbability laneChangeProbability(const LaneChangeProbability& modelBased,
                                                   const LaneChangeProbability& moveBased,
                                                   bool leftValid, bool rightValid){
    const double wModel = weightsToLCInfoSource(modelBased);
    const double wMove = weightsToLCInfoSource(moveBased);
    const double pL = leftValid ? wModel * modelBased.left + wMove * moveBased.left : 0.;
    const double pK = wModel * modelBased.keep + wMove * moveBased.keep;
    const double pR = rightValid ? wModel * modelBased.right + wMove * moveBased.right : 0.;
    const double sum = pL + pK + pR;
    return {pL / sum, pK / sum, pR / sum};
}

inline LaneChangeProbability moveBasedLaneChangeProbability(const double& signedDisToCenter, const double& vy){
    // output range -1 ~ 1, -1: right, 1: left, 0: keep
    const double output = 0.333 * signedDisToCenter + 0.6805 * vy - 0.01;
    const double qL = 0.8 * std::exp(-(output - 1.04) * (output - 1.04) / 0.055) +
                      0.8 * std::exp(-(output - 0.35) * (output - 0.35) / 0.055) +
                      0.65 * std::exp(-(output - 0.7) * (output - 0.7) / 0.07);
    const double qR = 0.75 * std::exp(-(output + 1.04) * (output + 1.04) / 0.055) +
                      0.75 * std::exp(-(output + 0.35) * (output + 0.35) / 0.055) +
                      0.6 * std::exp(-(output + 0.7) * (output + 0.7) / 0.07);
    const double qK = 2.5 * std::exp(-output * output / 0.055);
    const double sum = qL + qK + qR;
    return {qL / sum, qK / sum, qR / sum};
}

//inline double tCritical(const double& a, const double& d, const double& dm,
//                       const double& v1, const double& v2, const double& rho){
//    return std::max(((-a/dm - 1) * v1 + (1 - d/dm) * v2 - rho * d) / (a + d - d*d/dm + a*a/dm), 0.);
//}
//
//inline bool rssSafe(const IdmMatches& fs, const IdmMatch& b, const double& egoV, const RSSParam& p){
//    const double rTOther = p.rTimeOther;
//    const double rTEgo = p.rTimeEgo;
//    const double aMinBrake = p.aMinBrake;  // min brake force that the following vehicle can execute
//    const double aMaxBrake = p.aMaxBrake;  // max brake force that the front vehicle might execute
//    const double a = p.a;  // soft acceleration after merging
//    const double d = p.d;  // soft deceleration that the prioritized vehicle executes after perceiving merging behavior of ego
//    // check critical time in the future according to definition in page 75 of Diss Naumann
//    bool rssSafeF = true;
//    bool rssSafeB = true;
//    double egoAccReal = 0;
//    if (b.id >= 0){
//        const double tC = tCritical(a, d, -aMaxBrake, egoV, b.v, 1.0);
//        const double v1Tc = egoV + a * tC;
//        const double v2Tc = std::max(b.v - d * tC, 0.);
//        const double rssDisTC = std::max(v2Tc * rTOther + v2Tc * v2Tc / (-2 * aMinBrake) -
//                                                v1Tc * v1Tc / (-2 * aMaxBrake), 0.);
//        const double rssDisB = rssDisTC + b.v * tC - 0.5 * d * tC*tC - egoV * tC - 0.5 * a * tC*tC;
//        if (-b.dis < rssDisB){
//            rssSafeB = false;
//        }
//    }
//    if (!fs.empty()){
//        const auto f = fs[0];
//        if (f.id >= 0){
//            const double tC = tCritical(0., d, -aMaxBrake, f.v, egoV, 1.0);
//            const double v1Tc = f.v + a * tC;
//            const double v2Tc = std::max(egoV - d * tC, 0.);
//            const double rssDisTC = std::max(v2Tc * rTEgo + v2Tc * v2Tc / (-2 * aMinBrake) -
//                                             v1Tc * v1Tc / (-2 * aMaxBrake), 0.);
//            const double rssDisF = rssDisTC + egoV * tC - 0.5 * d * tC*tC - f.v * tC - 0.5 * a * tC*tC;
//            if (f.dis < rssDisF){ rssSafeF = false; }
//        }
//    }
//    return rssSafeB && rssSafeF;
//}

inline double yieldingPTTC(const double& ttc){
    if (ttc > 3.0 || ttc < 0.){
        return 0.9;
    }
    if (0.2 < ttc && ttc < 1.0){
        return 0.1;
    }
    if (0. <= ttc && ttc < 0.2){
        return 0.05;
    }
    return 0.1 + (ttc - 1.0) / 2.0 * 0.8;
}

inline double yieldingPTHW(const double& thw){
    if(thw > 3.0){
        return 0.9;
    }
    if(thw < 0.5){
        return 0.1;
    }
    return 0.1 + (thw - 0.5) / 2.5 * 0.8;
}

inline double yieldingProbability(const double& ttc, const double& thw, const double& d){
    if(d >= -10 && d <= 0){
        return 0.02;
    }
    if(d < -10){
        return 0.;
    }
    return std::min(yieldingPTHW(thw), yieldingPTTC(ttc));
}

inline double yieldingProbabilityLearned(const YieldingFeature& f){
    return 1 / (1 + std::exp(-(-0.5 + 1.81 * f.thw - 4.8 * f.ttc - 1.1 * f.acc)));
}

} // namespace mc_sim_highway

#endif // SRC_UTILITY_HPP
