#pragma once
#include <future>
#include "behaviors.hpp"

namespace mc_sim_highway{

struct SimParameter{
    double dt{0.3};
    int totalSteps{20};
    int minSteps{10};
};

struct StatusOneSim{
    bool fallBack;
    bool finish;
    double utility;
    double comfort;
    double utilityObjects;
    double comfortObjects;
    int finishStep;
};

struct BehaviorFeature{
    double fallBackRate;
    double successRate;
    double utility;
    double comfort;
    double expectedStepRatio;  // average_step / max_step
    double utilityObjects;
    double comfortObjects;
    int fromNSims;
};

struct Simulation{
    Simulation(const SimParameter& param, const MapMultiLane& map, const Agents& as): param_(param){
        env_ = Environment(map, as);
    }

    void reset(){
        env_.reset();
        step_ = 0;
    }

    bool setEgoAgentIdAndDrivingBehavior(const int& id, const std::string& direction, const int& targetGapId = -2){
        return env_.setEgoAgentIdAndDrivingBehavior(id, direction, targetGapId);
    }

    void randomizeAgentsParams(){
        env_.randomizeAgentsParams();
    }

    StatusOneSim run(){
        StatusOneSim status{};
        status.finishStep = param_.totalSteps;
        for(; step_ < param_.totalSteps; step_++){
            env_.step(param_.dt);
            if(env_.egoTargetReached()){
                status.finishStep = std::min(step_, status.finishStep);
                status.finish = true;
                if(step_ >= param_.minSteps){
                    break;
                }
            }
        }
        // minimum step of doing a free lane change with 1m/s -> 1.0s min lane change time
        status.finishStep = std::max(status.finishStep, int(1.0 / param_.dt));
        auto cuOthers = env_.meanComfortUtilityOthers();
        status.fallBack = env_.egoOnceFallBack();
        status.utility = env_.egoUtility();
        status.comfort = env_.egoComfort();
        status.comfortObjects = cuOthers.first;
        status.utilityObjects = cuOthers.second;
        return status;
    }

    BehaviorFeature runMultipleTimes(const int& n){
        BehaviorFeature feature{};
        feature.fromNSims = n;
        int numFallBack = 0;
        int numFinish = 0;
        int totalStep = 0;
        double utility = 0.;
        double comfort = 0.;
        double utilityObjects = 0.;
        double comfortObjects = 0.;
        for(int i = 0; i < n; i++){
            const auto status = run();
            if(status.fallBack){
                numFallBack++;
            }
            if(status.finish){
                numFinish++;
                totalStep = totalStep + status.finishStep;
            }
            utility = utility + status.utility;
            comfort = comfort + status.comfort;
            utilityObjects = utilityObjects + status.utilityObjects;
            comfortObjects = comfortObjects + status.comfortObjects;
            reset();
            randomizeAgentsParams();
        }
        feature.utility = utility / double(n);
        feature.comfort = comfort / double(n);
        feature.comfortObjects = comfortObjects / double(n);
        feature.utilityObjects = utilityObjects / double(n);
        feature.fallBackRate = double(numFallBack) / double(n);
        feature.successRate = double(numFinish) / double(n);
        feature.expectedStepRatio = double(std::max(totalStep, 1)) / double(std::max(param_.totalSteps * numFinish, 1));
        return feature;
    }

    ObjHistoryStatesMap agentsStatesHistory(){
        ObjHistoryStatesMap out;
        for(const auto& a: env_.agents){
            out.insert({a.first, a.second->statesHistory});
        }
        return out;
    }

    int step_{0};
    Environment env_;
    SimParameter param_;
};


void runMTimes(const int& n, BehaviorFeature& feature, const SimParameter& param,
               const MapMultiLane& map, const Agents& as, const int& egoId,
               const std::string& targetDirection, const int& targetGapId){
    feature.fromNSims = n;
    int numFallBack = 0;
    int numFinish = 0;
    int totalStep = 0;
    double utility = 0.;
    double comfort = 0.;
    double utilityObjects = 0.;
    double comfortObjects = 0.;
    auto sim = Simulation(param, map, as);
    bool set = sim.setEgoAgentIdAndDrivingBehavior(egoId, targetDirection, targetGapId);
    if(!set){
        feature.fromNSims = 0;
        return;
    }
    for(int i = 0; i < n; i++){
        const auto status = sim.run();
        if(status.fallBack){
            numFallBack++;
        }
        if(status.finish){
            numFinish++;
            totalStep = totalStep + status.finishStep;
        }
        utility = utility + status.utility;
        comfort = comfort + status.comfort;
        utilityObjects = utilityObjects + status.utilityObjects;
        comfortObjects = comfortObjects + status.comfortObjects;
        sim.reset();
        sim.randomizeAgentsParams();
    }
    feature.utility = utility / double(n);
    feature.comfort = comfort / double(n);
    feature.comfortObjects = comfortObjects / double(n);
    feature.utilityObjects = utilityObjects / double(n);
    feature.fallBackRate = double(numFallBack) / double(n);
    feature.successRate = double(numFinish) / double(n);
    feature.expectedStepRatio = double(std::max(totalStep, 1)) / double(std::max(param.totalSteps * numFinish, 1));
    feature.fromNSims = n;
}


BehaviorFeature runMultiThreads(const int& nEachThread, const SimParameter& param, const MapMultiLane& map,
                                const Agents& as, const int& egoId,
                                const std::string& targetDirection, const int& targetGapId = -2){
    int threadNumber = 8;
    std::vector<std::thread> workers;
    std::vector<BehaviorFeature> features;
    features.resize(threadNumber);
    try {
        for (int i = 0; i < threadNumber; i++){
            std::thread t(runMTimes, nEachThread, std::ref(features[i]), param, map, as, egoId, targetDirection, targetGapId);
            workers.push_back(std::move(t));
        }

        std::for_each(workers.begin(), workers.end(), [](std::thread &t){ t.join(); });
        double fallBackRate = 0.;
        double successRate = 0.;
        double utility = 0.;
        double comfort = 0.;
        double expectedStepRatio = 0.;
        double utilityObjects = 0.;
        double comfortObjects = 0.;
        int realValidThread = 0;
        for(auto& i: features){
            if(i.fromNSims > 0){
                fallBackRate = fallBackRate + i.fallBackRate;
                successRate = successRate + i.successRate;
                utility = utility + i.utility;
                comfort = comfort + i.comfort;
                expectedStepRatio = expectedStepRatio + i.expectedStepRatio;
                utilityObjects = utilityObjects + i.utilityObjects;
                comfortObjects = comfortObjects + i.comfortObjects;
                realValidThread++;
            }
        }

        return {fallBackRate / double(realValidThread),
                successRate / double(realValidThread),
                utility / double(realValidThread),
                comfort / double(realValidThread),
                expectedStepRatio / double(realValidThread),
                utilityObjects / double(realValidThread),
                comfortObjects / double(realValidThread), realValidThread * nEachThread};
    } catch (...) {
        return {0., 0., 0., 0., 0., 0., 0., 0};
    }
}


}