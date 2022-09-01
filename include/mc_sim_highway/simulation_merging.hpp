#ifndef SRC_SIMULATION_MERGING_HPP
#define SRC_SIMULATION_MERGING_HPP
#pragma once
#include <map>
#include <random>
#include <numeric>
#include <algorithm>
#include <thread>
#include "vehicle.hpp"

namespace mc_sim_highway{

struct PositionHistory{
    bool success;
    Points egoPositionHistory;
    ObjHistoryMap neighborPositionHistory;
};

class SimulationMerging{

    struct TerminationStatus{
        bool fallBack;
        bool collision;
        bool maxStepReached;
        bool finish;
        int totalStep;
        bool error;
    };

public:
    SimulationMerging() = default;
    SimulationMerging(const Vehicle& ego, const Vehicles& egoFront, const Vehicles& objects, const MapMerging& map, const RSSParam& p):
        map_(map), ego_(ego), egoCopy_(ego), egoFront_(egoFront), objects_(objects), objectsCopy_(objects), p_(p) {
        egoFrontCopy_ = egoFront;
        neighbor_ = relevantObjects();
        neighborEgoFront_ = relevantObjectsEgoFront();
        neighborCopy_ = neighbor_;
        neighborEgoFrontCopy_ = neighborEgoFront_;
        frontVehiclesCopy_ = frontVehicles_;
        frontVehiclesEgoFrontCopy_ = frontVehiclesEgoFront_;
        initialActionsCopy_ = initialActions_;
        initialActionsEgoFrontCopy_ = initialActionsEgoFront_;
    }

    void set(const Vehicle& ego, const Vehicles& egoFront, const Vehicles& objects, const MapMerging& map, const RSSParam& p){
        map_ = map;
        ego_ = ego;
        egoCopy_ = ego;
        egoFront_ = egoFront;
        egoFrontCopy_ = egoFront;
        objects_ = objects;
        p_ = p;
        neighbor_ = relevantObjects();
        neighborEgoFront_ = relevantObjectsEgoFront();
        neighborCopy_ = neighbor_;
        neighborEgoFrontCopy_ = neighborEgoFront_;
        frontVehiclesCopy_ = frontVehicles_;
        frontVehiclesEgoFrontCopy_ = frontVehiclesEgoFront_;
        initialActionsCopy_ = initialActions_;
        initialActionsEgoFrontCopy_ = initialActionsEgoFront_;
    }

    void reset(){
        step_ = 0;
        utility_ = 0.;
        comfort_ = 0.;
        history_.egoPositionHistory.clear();
        history_.neighborPositionHistory.clear();
        ego_ = egoCopy_;
        egoFront_ = egoFrontCopy_;
        objects_ = objectsCopy_;
        neighbor_ = neighborCopy_;
        neighborEgoFront_ = neighborEgoFrontCopy_;
        frontVehicles_ = frontVehiclesCopy_;
        frontVehiclesEgoFront_ = frontVehiclesEgoFrontCopy_;
        initialActions_ = initialActionsCopy_;
        initialActionsEgoFront_ = initialActionsEgoFrontCopy_;
    }
    void setPrintDebug(bool debug, int printId){
        printDebug_ = debug;
        printId_ = printId;
    }
    void setMaxStep(int maxStep){
        maxSteps_ = maxStep;
    }
    void setEgoFrontSimplePolicy(bool egoFrontSimplePolicy){
        egoFrontSimpleMergingPolicy_ = egoFrontSimplePolicy;
    }
    double utility() const{ return utility_; }
    double comfort() const{ return comfort_; }
    // return utility value between 0-1 at each step, penalize for v > vd as well
    double stepUtility() const{
        return ego_.v <= map_.vd ? ego_.v / map_.vd: std::max(1. - (ego_.v / map_.vd - 1.), 1.);
    }
    // return comfort value between 0-1 at each step
    double stepComfort(const double& ax) const{
        return ax >= 0 ? 1 + ax / p_.aMinBrake : 1 - ax / p_.aMinBrake;
    }

    double averageNormalizedAccObjects() const{
        if (neighbor_.empty()){
            return 0.;
        }
        double sumNormalizedAccObjs = 0.;
        for(const auto& obj: neighbor_){
//            std::cout << "id " << obj.first << " ";
//            for(auto& a: obj.second.accHistory){
//                std::cout << " " << a << " ";
//            }
//            std::cout << std::endl;
            double sumAccObj = std::accumulate(obj.second.accHistory.begin(), obj.second.accHistory.end(), 0.);
            double meanNormalizedAccObj = (sumAccObj / double(obj.second.accHistory.size()) - obj.second.idmP.accMin) /
                    (obj.second.idmP.accMax - obj.second.idmP.accMin);
            sumNormalizedAccObjs = sumNormalizedAccObjs + meanNormalizedAccObj;
//            std::cout << "id " << obj.first << " normalized acc: " << meanNormalizedAccObj << std::endl;
        }
        return sumNormalizedAccObjs / double(neighbor_.size());
    }

    bool onTargetLane(const Vehicle& car) const{
        return car.y >= 0.5 * map_.targetLane.p1.y;
    }

    std::map<int, Vehicle> relevantObjects(){
        std::map<int, Vehicle> out;
        for (const auto& o: objects_){
            if (onTargetLane(o) && o.x > ego_.x + 50){
                out.insert({o.id, o});
                frontVehicles_.push_back(o.id);
            }
        }
        int numPossibleYieldingObjects = 0;
        for (auto i = objects_.rbegin(); i != objects_.rend(); ++ i) {
            if (onTargetLane(*i) && i->x <= ego_.x + 50 && numPossibleYieldingObjects < 4){
                initialActions_.push_back(i->id);
                numPossibleYieldingObjects++;
                out.insert({i->id, *i});
            }
        }
        return out;
    }

    std::map<int, Vehicle> relevantObjectsEgoFront(){
        std::map<int, Vehicle> out;
        if(egoFront_.empty()){
            return out;
        }
        const auto egoFront = egoFront_[0];
        for (const auto& o: objects_){
            if (onTargetLane(o) && o.x > egoFront.x + 50){
                out.insert({o.id, o});
                frontVehiclesEgoFront_.push_back(o.id);
            }
        }
        int numPossibleYieldingObjects = 0;
        for (auto i = objects_.rbegin(); i != objects_.rend(); ++ i) {
            if (onTargetLane(*i) && i->x <= egoFront.x + 50 && numPossibleYieldingObjects < 4){
                initialActionsEgoFront_.push_back(i->id);
                numPossibleYieldingObjects++;
                out.insert({i->id, *i});
            }
        }
        return out;
    }

    int closestGap(){
        // choose the gap that can be reached fastest using constant acc/dcc with +-2m/s^2
        std::map<int, double> gapWithTime;
        for(const auto& id: initialActions_){
            const auto back =  neighbor_.find(id)->second;
            if (id == 0){
                // the first gap, overtake all
                gapWithTime[id] = timeToReachGap(back.x + 0.5 * (back.l + ego_.l), back.v, -1e10, -1e10, ego_.x, ego_.v);
            }else{
                const auto front = neighbor_.find(id - 1)->second;
                gapWithTime[id] = timeToReachGap(back.x + 0.5 * (back.l + ego_.l), back.v,
                                                 front.x - 0.5 * (front.l + ego_.l), front.v, ego_.x, ego_.v);
            }
        }
        // merge into last gap
        if(!initialActions_.empty()){
            const auto front =  neighbor_.find(initialActions_.back())->second;
            gapWithTime[-1] = timeToReachGap(-1e10, -1e10, front.x - 0.5 * (front.l + ego_.l), front.v, ego_.x, ego_.v);
        }else{
            gapWithTime[-1] = 0.;
        }
        int actionId = -2;
        double minTime = 1e10;
        for(const auto& gap: gapWithTime){
//            std::cout << "gap: " << gap.first << " time: " << gap.second << std::endl;
            if(gap.second < minTime){
                actionId = gap.first;
                minTime = gap.second;
            }
            if(gap.second == minTime){
                // with the same approaching time, always the front gap is preferred
                if(gap.first != -1 && gap.first < actionId){
                    actionId = gap.first;
                    minTime = gap.second;
                }
            }
        }
        return actionId;
    }

    int closestGapEgoFront(const Vehicle& egoFront){
        // choose the gap that can be reached fastest using constant acc/dcc with +-2m/s^2
        std::map<int, double> gapWithTime;
        for(const auto& id: initialActionsEgoFront_){
            const auto back =  neighborEgoFront_.find(id)->second;
            if (id == 0){
                // the first gap, overtake all
                gapWithTime[id] = timeToReachGap(back.x + 0.5 * (back.l + egoFront.l), back.v, -1e10, -1e10, egoFront.x, egoFront.v);
            }else{
                const auto front = neighborEgoFront_.find(id - 1)->second;
                gapWithTime[id] = timeToReachGap(back.x + 0.5 * (back.l + egoFront.l), back.v,
                                                 front.x - 0.5 * (front.l + egoFront.l), front.v, egoFront.x, egoFront.v);
            }
        }
        // merge into last gap
        if(!initialActionsEgoFront_.empty()){
            const auto front =  neighborEgoFront_.find(initialActionsEgoFront_.back())->second;
            gapWithTime[-1] = timeToReachGap(-1e10, -1e10, front.x - 0.5 * (front.l + egoFront.l), front.v, egoFront.x, egoFront.v);
        }else{
            gapWithTime[-1] = 0.;
        }
        int actionId = -2;
        double minTime = 1e10;
        for(const auto& gap: gapWithTime){
            if(gap.second < minTime){
                actionId = gap.first;
                minTime = gap.second;
            }
            if(gap.second == minTime){
                // with the same approaching time, always the front gap is preferred
                if(gap.first != -1 && gap.first < actionId){
                    actionId = gap.first;
                    minTime = gap.second;
                }
            }
        }
        return actionId;
    }

    double lateralVelocity(const IdmMatches& front, const IdmMatch& back, const Vehicle& ego){
        // fs: [---ego---fs[0]---fs[1]---...]
        // try to always approach the boundary
        if(ego.finishedMerging(map_)){
            double signedDistance = map_.targetLane.p1.y - ego.y;
            double vyAbs = std::max(std::abs(signedDistance) * 1.0, 0.3);
            return signedDistance > 0 ? vyAbs : -vyAbs;
        }
        bool farFromApproachingMerging = 0.5 * map_.targetLane.p1.y - ego.y > 0.5 * ego.d + 0.5;
        bool notFarFromApproachingMerging = 0.5 * map_.targetLane.p1.y - ego.y <= 0.5 * ego.d + 0.5 &&
                0.5 * map_.targetLane.p1.y - ego.y >= 0.5 * ego.d;
        rssSafeForLaneChange_[ego.id] = rssSafeRelax(front, back, ego.v, map_.vd, p_, ego.idmP);
        if(farFromApproachingMerging || rssSafeForLaneChange_[ego.id]){
            return std::min(0.17 * ego.v, 1.0);
        }
        if(notFarFromApproachingMerging){
            if(rssSafeForLaneChange_[ego.id]){
                return std::min(0.17 * ego.v, 1.0);
            }else{
                return 0.;
            }
        }
        return -std::min(0.17 * ego.v, 1.0);
    }

    TerminationStatus checkTermination(const Action& action){
        TerminationStatus status{};
        // fall back: front vehicle stops already and RSS distance is kept
        status.fallBack = (!ego_.fallBackExist(map_, p_.rTimeEgo)) && !rssSafeForLaneChange_[ego_.id] && 0.5 * map_.targetLane.p1.y - ego_.y > 0.5 * ego_.d;
//        (ego_.timeToFinishMerging(map_, action.vy) > ego_.timeToReachMergingLaneEnd(map_))
        status.collision = false;
//        for(const auto& obj: neighbor_){
//            if(ego_.collideWith(obj.second)){
//                status.collision = true;
//                break;
//            }
//        }
        status.maxStepReached = step_ >= maxSteps_;
        status.finish = ego_.finishedMerging(map_);
        status.totalStep = step_;
        status.error = false;
        return status;
    }

    static bool terminated(const TerminationStatus& status){
        return status.finish || status.maxStepReached || status.collision || status.fallBack;
    }
    const PositionHistory& positionHistory(){ return history_; }
    bool actionWithId(const int& id, Action& action, bool printDebug=false);
    bool actionWithIdEgoFront(const int& id, Action& action, Vehicle& egoFront, bool printDebug=false);
    void step(const Action& action);
    TerminationStatus runOneSimulation(const int& initialEgoAction);
    TerminationStatus runOneSimulationWithoutHistory(const int& initialEgoAction);


private:
    MapMerging map_{{{0., 0.}, {150., 0.}}, {{0., 3.}, {400., 3.}}, 80/3.6};
    Vehicle ego_{0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0}};
    Vehicle egoCopy_{0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0}};
    Vehicles egoFront_;
    Vehicles egoFrontCopy_;
    Vehicles objects_;
    Vehicles objectsCopy_;
    std::vector<int> frontVehicles_;
    std::vector<int> frontVehiclesCopy_;
    std::vector<int> frontVehiclesEgoFront_;
    std::vector<int> frontVehiclesEgoFrontCopy_;
    std::vector<int> initialActions_;  // list of id of Action object
    std::vector<int> initialActionsCopy_;
    std::vector<int> initialActionsEgoFront_;
    std::vector<int> initialActionsEgoFrontCopy_;
    std::map<int, Vehicle> neighbor_;
    std::map<int, Vehicle> neighborCopy_;
    std::map<int, Vehicle> neighborEgoFront_;
    std::map<int, Vehicle> neighborEgoFrontCopy_;
    PositionHistory history_;
    bool egoFrontSimpleMergingPolicy_{true};

    int step_{0};
    int maxSteps_{20};
    int printId_{-1};
    std::map<int, bool> rssSafeForLaneChange_;
    bool printDebug_{false};
    double dt_{0.3};
    double utility_{0.};
    double comfort_{0.};
    RSSParam p_{0, 0, 0, 0, 0, 0};
};

struct Indicator{
    double fallBackRate;
    double successRate;
    double utility;
    double comfort;
    double expectedStepRatio;  // average_step / max_step
    double normalizedAccObjects;
    bool error;
};

PositionHistory simulationMergingOnce(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                                      const MapMerging& map, const int& actionId, const RSSParam& p,
                                      bool printDebug, int printId){
    auto simulation = SimulationMerging(ego, egoFront, objects, map, p);
    simulation.setPrintDebug(printDebug, printId);
    auto status = simulation.runOneSimulation(actionId);
    auto history = simulation.positionHistory();
    history.success = status.finish;
    return history;
}

Indicator simulationMerging(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                            const MapMerging& map, const int& actionId, const RSSParam& p, const int& maxStep){
    const int simulationNum = 500;
    int numSuccess = 0;
    int numFallBack = 0;
    double utility = 0.;
    double comfort = 0.;
    double meanNormalizedAccObjects = 0.;
    int totalStep = 0;
    auto simulation = SimulationMerging(ego, egoFront, objects, map, p);
    simulation.setMaxStep(maxStep);
    for(int i = 0; i < simulationNum; i++){
        const auto status = simulation.runOneSimulationWithoutHistory(actionId);
        if(status.error) {
            return {1., 0., 0., 0., 1., 0., true};
        }
        utility = utility + simulation.utility();
        comfort = comfort + simulation.comfort();
        meanNormalizedAccObjects = meanNormalizedAccObjects + simulation.averageNormalizedAccObjects();
        if(status.finish){
            numSuccess++;
            totalStep = totalStep + status.totalStep;
        }
        else if(status.fallBack) { numFallBack++; }
        simulation.reset();
    }
//    std::cout << "fall back time: " << numFallBack << std::endl;
    return {double(numFallBack) / double(simulationNum),
            double(numSuccess) / double(simulationNum),
            utility / double(simulationNum),
            comfort / double(simulationNum),
            double(std::max(totalStep, 1)) / double(std::max(maxStep * numSuccess, 1)),
            meanNormalizedAccObjects / double(simulationNum),
            false};
}

void simulationMergingOneThread(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                                const MapMerging& map, const int& actionId, const RSSParam& p, const int& maxStep,
                                Indicator& Indicator, bool egoFrontSimplePolicy){
    const int simulationNum = 70;
    int numSuccess = 0;
    int numFallBack = 0;
    double utility = 0.;
    double comfort = 0.;
    double meanNormalizedAccObjects = 0.;
    int totalStep = 0;
    auto simulation = SimulationMerging(ego, egoFront, objects, map, p);
    simulation.setEgoFrontSimplePolicy(egoFrontSimplePolicy);
    simulation.setMaxStep(maxStep);
    for(int i = 0; i < simulationNum; i++){
        const auto status = simulation.runOneSimulationWithoutHistory(actionId);
        if(status.error) {
            Indicator = {1., 0., 0., 0., 1., 0., true};
            return;
        }
        utility = utility + simulation.utility();
        comfort = comfort + simulation.comfort();
        meanNormalizedAccObjects = meanNormalizedAccObjects + simulation.averageNormalizedAccObjects();
        if(status.finish){
            numSuccess++;
            totalStep = totalStep + status.totalStep;
        }
        else if(status.fallBack) { numFallBack++; }
        simulation.reset();
    }
//    std::cout << "fall back time: " << numFallBack << std::endl;
    Indicator = {double(numFallBack) / double(simulationNum),
                 double(numSuccess) / double(simulationNum),
                 utility / double(simulationNum),
                 comfort / double(simulationNum),
                 double(std::max(totalStep, 1)) / double(std::max(maxStep * numSuccess, 1)),
                 meanNormalizedAccObjects / double(simulationNum),
                 false};
}

Indicator simulationMergingMultiThread(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                                       const MapMerging& map, const int& actionId, const RSSParam& p, const int& maxStep,
                                       bool egoFrontSimplePolicy=true){
    int threadNumber = 6;
    Indicator indicator1{};
    Indicator indicator2{};
    Indicator indicator3{};
    Indicator indicator4{};
    Indicator indicator5{};
    Indicator indicator6{};
//    Indicator indicator7{};
//    Indicator indicator8{};
    std::thread t1(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator1), egoFrontSimplePolicy);
    std::thread t2(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator2), egoFrontSimplePolicy);
    std::thread t3(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator3), egoFrontSimplePolicy);
    std::thread t4(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator4), egoFrontSimplePolicy);
    std::thread t5(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator5), egoFrontSimplePolicy);
    std::thread t6(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator6), egoFrontSimplePolicy);
//    std::thread t7(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator7), egoFrontSimplePolicy);
//    std::thread t8(simulationMergingOneThread, objects, egoFront, ego, map, actionId, p, maxStep, std::ref(indicator8), egoFrontSimplePolicy);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
//    t7.join();
//    t8.join();
    double fallBackRate = 0.;
    double successRate = 0.;
    double utility = 0.;
    double comfort = 0.;
    double expectedStepRatio = 0.;
    double meanNormalizedAccObjects = 0.;
    bool error = false;
    std::vector<Indicator> indicators{indicator1, indicator2, indicator3, indicator4,
                                      indicator5, indicator6};
    for(auto& i: indicators){
        fallBackRate = fallBackRate + i.fallBackRate;
        successRate = successRate + i.successRate;
        utility = utility + i.utility;
        comfort = comfort + i.comfort;
        expectedStepRatio = expectedStepRatio + i.expectedStepRatio;
        meanNormalizedAccObjects = meanNormalizedAccObjects + i.normalizedAccObjects;
        error = error || i.error;
    }
    return {fallBackRate / double(threadNumber),
            successRate / double(threadNumber),
            utility / double(threadNumber),
            comfort / double(threadNumber),
            expectedStepRatio / double(threadNumber),
            meanNormalizedAccObjects / double(threadNumber),
            error};
}

Action getMergingAction(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                        const MapMerging& map, const int& actionId, const RSSParam& p){
    auto simulation = SimulationMerging(ego, egoFront, objects, map, p);
    Action action{};
    if (simulation.actionWithId(actionId, action, false)){
        return action;
    }
    return {ego.idmP.aEmergency, 0};
}

Action getMergingActionClosestGap(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                                  const MapMerging& map, const RSSParam& p){
    auto simulation = SimulationMerging(ego, egoFront, objects, map, p);
    Action action{};
    if (simulation.actionWithId(simulation.closestGap(), action, false)){
        return action;
    }
    return {ego.idmP.aEmergency, 0};
}

int getMergingActionClosestGapId(const Vehicles& objects, const Vehicles& egoFront, const Vehicle& ego,
                                 const MapMerging& map, const RSSParam& p){
    auto simulation = SimulationMerging(ego, egoFront, objects, map, p);
    return simulation.closestGap();
}

}  // namespace mc_sim_highway

#endif // SRC_SIMULATION_MERGING_HPP
