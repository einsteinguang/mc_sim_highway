#pragma once
#include "types.hpp"

namespace mc_sim_highway{

struct Environment;
struct Agent;

using Agents = std::vector<Agent>;
using AgentPointer = std::shared_ptr<Agent>;
using AgentsPointer = std::vector<AgentPointer>;

inline Action idmBehavior(const Environment& env, AgentPointer& agent, bool probabilistic);
inline Action idmBehaviorAndYieldingToMergingAgents(const Environment& env, AgentPointer& agent, bool probabilistic);
inline Action mergingBehaviorClosestGap(const Environment& env, AgentPointer& agent);
inline Action laneChangeBehaviorMOBIL(const Environment& env, AgentPointer& agent);
inline Action customizedLaneChangeBehavior(const Environment& env, AgentPointer& agent);
inline Action customizedMergingBehavior(const Environment& env, AgentPointer& agent);

struct Neighbor{
    AgentPointer lFF = nullptr;
    AgentPointer lF = nullptr;
    AgentPointer lB = nullptr;
    AgentPointer lBB = nullptr;
    AgentPointer f = nullptr;
    AgentPointer b = nullptr;
    AgentPointer rFF = nullptr;
    AgentPointer rF = nullptr;
    AgentPointer rB = nullptr;
    AgentPointer rBB = nullptr;

    AgentsPointer left(){
        AgentsPointer out;
        for(auto& a: {lBB, lB, lF, lFF}){
            if(a){
                out.push_back(a);
            }
        }
        return out;
    }

    AgentsPointer possibleLeft(){
        return {lBB, lB, lF, lFF};
    }

    AgentsPointer right(){
        AgentsPointer out;
        for(auto& a: {rBB, rB, rF, rFF}){
            if(a){
                out.push_back(a);
            }
        }
        return out;
    }

    AgentsPointer possibleRight(){
        return {rBB, rB, rF, rFF};
    }
};

struct Agent{
    Agent(const int& id, const double& x, const double& y, const double& v, const double& vy, const double& vd,
          const double& a, const double& l, const double& d,
          const IdmParam& idmP, const RSSParam& rssP, const YieldingParam& yp,
          std::string bm):
          id(id), x(x), y(y), v(v), vy(vy), desiredV(vd), a(a), l(l), d(d), idmP(idmP), rssP(rssP),
          behaviorModel(std::move(bm)){
        behaviorModelHistory = {behaviorModel};
        statesHistory = {{x, y, v, a}};
        yieldingModel = EstimatedYieldingModel(yp);
        rssPUnsafe = rssP;
        rssPUnsafe.aMinBrake = -10.;
        rssPUnsafe.aMaxBrake = -1.;
        idmPYielding = idmP;
        if(l < 7){
            idmPYielding.accMin = idmP.accMin + 1.0;
            idmPYielding.accMax = idmP.accMax - 0.8;
        }
        aggressiveAgent = (double)rand() / RAND_MAX > 0.8 ? true : false;
    }

    void setBehaviorModel(const std::string& model){
        behaviorModel = model;
        behaviorModelHistory.push_back(model);
    }

    void setDesiredV(const double& vd){
        desiredV = vd;
    }

    void setMoveBasedLaneChangeP(const LaneChangeProbability& p){
        moveBasedLCP = p;
    }

    void resetBehaviorModel(const std::string& model){
        behaviorModel = model;
        behaviorModelHistory = {behaviorModel};
    }

    void resetAgressiveness(){
        aggressiveAgent = (double)rand() / RAND_MAX > 0.8 ? true : false;
    }

    void step(const Action& action, const double& dt){
        v = std::max(v + action.ax * dt, 0.);
        x = x + v * dt;
        y = y + action.vy * dt;
        vy = action.vy;
        if(v == 0.){ a = 0.; } else{ a = action.ax; }
        statesHistory.push_back({x, y, v, a});
    }
    int id;
    double x;
    double y;
    double v;
    double vy;
    double a;
    double l;
    double d;
    double desiredV;
    LaneChangeProbability moveBasedLCP;

    // parameters only for ego vehicle
    bool isEgo{false};
    bool targetReached{false};
    int targetGapId{-2};
    int targetLaneId{-1};
    std::string direction{"keep_lane"};
    // parameters for mobil lane change vehicles
    std::string commitToDecision{"not_decided"};
    int commitKeepLaneStep{0};
    // possibility to be an agent that do lc unsafely, 10%
    bool aggressiveAgent{false};

    IdmParam idmP;
    IdmParam idmPYielding;
    RSSParam rssP;
    RSSParam rssPUnsafe;
    std::string behaviorModel;
    std::vector<std::string> behaviorModelHistory;
    States statesHistory;
    std::vector<double> thwHistory;
    std::vector<int> corridorIdHistory;
    EstimatedYieldingModel yieldingModel;
    std::unordered_map<int, YieldingIntention> yieldingIntentionToOthers;
};


struct MatchedAgent{
    AgentPointer agent;
    int corridorId;
};
using MatchedAgents = std::vector<MatchedAgent>;

}
