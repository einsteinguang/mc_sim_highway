#ifndef SRC_TYPES_HPP
#define SRC_TYPES_HPP
#pragma once
#include <iostream>     // std::cout
#include <algorithm>    // std::max
#include <utility>
#include <vector>
#include <cmath>
#include <chrono>
#include <optional>
#include <unordered_map>
#include  <random>
#include  <iterator>

namespace mc_sim_highway{

template<typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));
    return start;
}

template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
}

struct Point{
    Point(const double& x, const double& y): x(x), y(y){}
    double x;
    double y;
};

struct State{
    double x;
    double y;
    double v;
    double a;
};
using Points = std::vector<Point>;
using States = std::vector<State>;
using ObjHistoryMap = std::map<int, Points>;
using ObjHistoryStatesMap = std::map<int, States>;

struct Line{
    Line(const Point& p1, const Point& p2): p1(p1), p2(p2){}
    Point p1;
    Point p2;
};

struct Corridor{
    Corridor() = default;
    Corridor(const int& id, std::string type, const Line& l, const Line& r, const Line& m, const double& vLimit):
    id(id), type(std::move(type)), l(l), r(r), m(m), vLimit(vLimit){}
    int id;
    std::optional<int> lId;
    std::optional<int> rId;
    std::string type;
    Line l;
    Line r;
    Line m;
    double vLimit;

    void setLeftAndRightCorridorIdWrapper(const int& leftId,
                                          const int& rightId){
        std::optional<int> leftIdReal = leftId;
        std::optional<int> rightIdReal = rightId;
        if(leftId < 0){
            leftIdReal = std::nullopt;
        }
        if(rightId < 0){
            rightIdReal = std::nullopt;
        }
        setLeftAndRightCorridorId(leftIdReal, rightIdReal);
    }

    void setLeftAndRightCorridorId(const std::optional<int>& leftId,
                                   const std::optional<int>& rightId){
        lId = leftId;
        rId = rightId;
    }

    bool within(const double& x, const double& y) const{
        return y <= l.p1.y && y >= r.p1.y;
//        return y <= l.p1.y && y >= r.p1.y && x <= m.p2.x && x >= m.p1.x;
    }

    double distanceToCorridorEnd(const double& x) const{
        return m.p2.x - x;
    }

    double signedDistanceToCenter(const double& y) const{
        return y - m.p1.y;
    }

    double signedDistanceToLeftBorder(const double& y, const double& w) const{
        return y + 0.5 * w - l.p1.y;
    }

    double signedDistanceToRightBorder(const double& y, const double& w) const{
        return y - 0.5 * w - r.p1.y;
    }

    bool reachCorridorBegin(const double& x) const{
        return x >= m.p1.x;
    }

    bool fallBackExist(const double& x, const double& v, const double& length, const double& dcc, const double& rt) const{
        const double disToStop = v * rt + v * v / (-2 * dcc);
        return disToStop <= m.p2.x - x - 15.;
    }
};
using Corridors = std::vector<Corridor>;

struct Action{
    double ax;
    double vy;
};

struct ActionWithType{
    double ax;
    double vy;
    std::string commitToDecision;
    int commitKeepLaneStep;
    int targetLaneId;
};

struct IdmMatch{
    int id;  // valid id >= 0
    double dis;  // dis is pure distance, front > 0, back < 0
    double v;
};
using IdmMatches = std::vector<IdmMatch>;

struct IdmMatchFull{
    int id;  // valid id >= 0
    double dis;  // dis is relative center distance, front > 0, back < 0
    double v;
    double a;
    double l;
    double w;
};
using IdmMatchPtr = std::shared_ptr<IdmMatchFull>;
using IdmMatchesPtr = std::vector<IdmMatchPtr>;

struct MapMerging{
    MapMerging(const Line& line1, const Line& line2, const double& vd):
    mergingLane(line1), targetLane(line2), vd(vd){}
    Line mergingLane{{0., 0.}, {150., 0.}};
    Line targetLane{{0., 3.}, {400., 3.}};
    double vd{80 / 3.6};
};

struct MapMultiLane{
    MapMultiLane() = default;
    explicit MapMultiLane(const Corridors& cs){
        for(const auto& c: cs){
            corridors.insert({c.id, c});
            corridorsByType.insert({c.type, {}});
            corridorsByType.find(c.type)->second.push_back(c);
        }
        initialCorridors = cs;
        setLeftAndRightCorridors();
    }

    bool hasExitLane() const{
        return corridorsByType.find("exit") != corridorsByType.end();
    }

    void setLeftAndRightCorridors(){
        std::sort(initialCorridors.begin(), initialCorridors.end(),
                  [](const Corridor& a, const Corridor& b)->bool{ return a.m.p1.y < b.m.p1.y;});
        for(auto& c: corridors){
            std::optional<int> lId = std::nullopt;
            std::optional<int> rId = std::nullopt;
            for(const auto& co: initialCorridors){
                if(co.m.p1.y > c.second.m.p1.y){
                    lId = co.id;
                    break;
                }
            }
            for(auto it = initialCorridors.rbegin(); it != initialCorridors.rend(); ++it){
                if(it->m.p1.y < c.second.m.p1.y){
                    rId = it->id;
                    break;
                }
            }
            c.second.setLeftAndRightCorridorId(lId, rId);
        }
        for(const auto& c: corridors){
            if(!c.second.lId){
                noTruckLane = c.first;
            }
        }
    }

    std::optional<Corridor> matchedCorridor(const double& x, const double& y) const{
        for(const auto& c: corridors){
            if(c.second.within(x, y)){
                return c.second;
            }
        }
        return std::nullopt;
    }

    bool matched(const int& corridorId, const double& x, const double& y) const{
        return corridors.find(corridorId)->second.within(x, y);
    }
    Corridors initialCorridors;
    int noTruckLane{-1};
    std::unordered_map<int, Corridor> corridors;
    std::unordered_map<std::string, Corridors> corridorsByType;
};


struct YieldingFeature{
    double thw;  // d / ego_v, represents no unit distance
    double ttc;  // (ego_v - obj_v) / ego_v, represents thw changing rate
    double acc;  // acc of ego, represents wish of ego vehicle to cooperate
};

struct YieldingParam{
    void randomize(){
        politenessFactor = politenessFactor + 0.5 * ((double) rand() / (RAND_MAX) - 0.5);
        deltaA = deltaA + 0.5 * ((double) rand() / (RAND_MAX) - 0.5);
        biasA = biasA + 0.5 * ((double) rand() / (RAND_MAX) - 0.5);
    }
    double bias{-0.5};
    double a{1.81};
    double b{-4.8};
    double c{-1.1};

    // learn parameter to fit: 12704 89736 0.14157082999019346 from highD
    double politenessFactor{0.9};  // For Mobil lane change model
    double deltaA{0.5};  // changing lane threshold
    double biasA{0.51};  // bias for asymmetric europe lane change rule
};

struct YieldingIntention{
    YieldingIntention(const int& id, const double& initialP,
                      const double& changingIntentionThreshold=0.625,
                      const std::optional<int>& randomIntentionSeed=std::nullopt):
    id(id), initialP(initialP), changingIntentionThreshold(changingIntentionThreshold){
        if(randomIntentionSeed){
            srand(randomIntentionSeed.value());
        }
        yielding = rand() % 100 <= initialP * 100;
    }
    int id;
    double initialP;
    double changingIntentionThreshold;
    bool yielding;

    void updateIntention(const double& newP){
        if(!yielding){
            if((1 - newP) <= changingIntentionThreshold * (1 - initialP)){
                yielding = true;
            }
        } else{
            if(newP <= changingIntentionThreshold * initialP){
                yielding = false;
            }
        }
    }
};

class EstimatedYieldingModel{
public:
    EstimatedYieldingModel() = default;
    explicit EstimatedYieldingModel(const YieldingParam& param): param(param){}

    double logistic_function(const YieldingFeature& f) const{
        const double r = std::max(std::min(100., (param.bias + param.a * f.thw + param.b * f.ttc + param.c * f.acc)), -100.);
        return 1 / (1 + std::exp(-r));
    }

    virtual double yieldingProbability(const double& matchDis,
                                       const double& matchL,
                                       const double& matchV,
                                       const double& egoL,
                                       const double& egoV,
                                       const double& initialEgoA) const{
        // yielding p of ego to idm match
        if(matchDis + 0.5 * (egoL + matchL) < 0){
            return 0.;
        }
        const double disPure = matchDis - 0.5 * (egoL + matchL);
        const double thw = disPure / std::max(egoV, 1e-10);
        const double thwPrime = (egoV - matchV) / std::max(1e-3, egoV);
        return logistic_function({thw, thwPrime, initialEgoA});
    }
    YieldingParam param;
};

struct IdmParam{
    IdmParam() = default;
    IdmParam(const double& am, const double& amin, const double& minD,
             const double& aCom, const double& aE, const double& t):
    accMax(am), accMin(amin), minDis(minD), accCom(aCom), aEmergency(aE), thw(t){}

    void randomize(){
        accMax = accMax + ((double) rand() / (RAND_MAX) - 0.5);
        accMin = accMin + ((double) rand() / (RAND_MAX) - 0.5);
        minDis = minDis + ((double) rand() / (RAND_MAX) - 0.5);
        accCom = accCom + ((double) rand() / (RAND_MAX) - 0.5);
        thw = thw + ((double) rand() / (RAND_MAX) - 0.5);;
    }
    double accMax = 1.5;
    double minDis = 2.0;
    double accMin = -4.0;
    double accCom = -2.0;
    double aEmergency = -7.0;
    double thw = 1.5;
};

struct RSSParam{
    RSSParam() = default;
    RSSParam(const double& rTOther, const double& rTEgo, const double& aMin,
             const double& aMax, const double& aSoft, const double& dSoft):
            rTimeOther(rTOther), rTimeEgo(rTEgo), aMinBrake(aMin), aMaxBrake(aMax), a(aSoft), d(dSoft){}
    void randomize(){
        std::vector<std::pair<double, double>> brakeTemplates = {{-8., -10.}, {-6., -6.}, {-6., -8.}};
        auto pair = select_randomly(brakeTemplates.begin(), brakeTemplates.end());
        aMinBrake = pair->first;
        aMaxBrake = pair->second;
//        rTimeOther = rTimeOther + 0.5 * ((double) rand() / (RAND_MAX) - 0.5);
//        rTimeEgo = rTimeEgo + 0.25 * ((double) rand() / (RAND_MAX) - 0.5);
//        aMinBrake = aMinBrake + ((double) rand() / (RAND_MAX) - 0.5);
//        aMaxBrake = aMaxBrake + ((double) rand() / (RAND_MAX) - 0.5);
//        a = a + ((double) rand() / (RAND_MAX) - 0.5);
//        d = d + ((double) rand() / (RAND_MAX) - 0.5);
    }
    double rTimeOther = 0.7;
    double rTimeEgo = 0.4;
    double aMinBrake = -8.0; // min brake force that the following vehicle can execute
    double aMaxBrake = -10.;  // max brake force that the front vehicle might execute
    double a = 1.8;  // soft acceleration after merging
    double d = 1.2;  // soft deceleration that the prioritized vehicle executes after perceiving merging behavior of ego
};

struct LaneChangeProbability{
    double left;
    double keep;
    double right;

    double maxLCP() const{
        return std::max(left, right);
    }
};

} // namespace mc_sim_highway

#endif // SRC_TYPES_HPP
