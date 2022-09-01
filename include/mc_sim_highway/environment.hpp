#pragma once
#include "types.hpp"
#include "agent.hpp"

namespace mc_sim_highway{



struct Environment{
    Environment() = default;
    Environment(const MapMultiLane& map, const Agents& as): map(map){
        for(const auto& a: as){
            agents.insert({a.id, std::make_shared<Agent>(a)});
        }
        matchAgentsOnCorridor();
        for(const auto& aOnC: sortedAgentsOnCorridor){
            cachedSortedAgentsOnCorridor.insert({aOnC.first, {}});
            for(const auto& a: aOnC.second){
                cachedSortedAgentsOnCorridor.find(aOnC.first)->second.push_back(*a);
            }
        }
    }

    void reset(){
        step_ = 0;
        agents.clear();
        sortedAgentsOnCorridor.clear();
        for(const auto& aOnC: cachedSortedAgentsOnCorridor){
            AgentsPointer aPointers;
            for(const auto& a: aOnC.second){
                auto aPtr = std::make_shared<Agent>(a);
                aPointers.push_back(aPtr);
                agents.insert({a.id, aPtr});
            }
            sortedAgentsOnCorridor.insert({aOnC.first, aPointers});
        }
        if(egoId > 0){
            setEgoAgentIdAndDrivingBehavior(egoId, egoDirection, egoTargetGapId);
        }
    }

    void randomizeAgentsParams(){
        for(auto& a: agents){
            if(a.first != egoId){
                a.second->resetAgressiveness();
//                a.second->idmP.randomize();
//                a.second->rssP.randomize();
//                a.second->yieldingModel.param.randomize();
            }
        }
    }

    bool setEgoAgentIdAndDrivingBehavior(const int& id, const std::string& direction, const int& targetGapId = -2){
        // return false if the lane change direction is not valid
        auto ego = agents.find(id);
        if(ego != agents.end()){
            ego_ = ego->second;
            ego->second->isEgo = true;
            ego->second->direction = direction;
            ego->second->targetReached = false;
            ego->second->targetGapId = targetGapId;
            egoId = id;
            egoDirection = direction;
            egoTargetGapId = targetGapId;
            if(ego->second->corridorIdHistory.empty()){
                std::cout << "Ego with id: " << id << " is not matched to any corridor!" << std::endl;
                return false;
            }
            const auto corridor = map.corridors.find(ego->second->corridorIdHistory.back())->second;
            if(direction == "left"){
                if(corridor.lId) {
                    ego->second->targetLaneId = *corridor.lId;
                    return true;
                }
                return false;
            }
            if(direction == "right"){
                if(corridor.rId) {
                    ego->second->targetLaneId = *corridor.rId;
                    return true;
                }
                return false;
            }
            if(direction == "keep_lane" || direction == "acc" || direction == "dcc"){
                ego->second->targetLaneId = ego->second->corridorIdHistory.back();
                return true;
            }
            return false;
        }else{
            std::cout << "No ego agent with id: " << id << std::endl;
            return false;
        }
    }

    bool egoTargetReached() const{
        // terminate if ego on target lane id
        if(egoId < 0){
            return false;
        }
        return ego_->targetReached;
    }

    bool egoOnceFallBack() const{

        if(egoId < 0){
            return false;
        }
//        for(const auto& s: ego_->statesHistory){
//            std::cout << "acc: " << s.a << " v " << s.v << std::endl;
//        }
        if (ego_->behaviorModelHistory[0] == "merging" || ego_->behaviorModelHistory[0] == "exit"){
            // return True when ego stops at end of the lane -> for merging and exit behavior
            if(ego_->v < 2.0){
                return true;
            }
        }else{
            // return true once ego have longer than 2 steps minBrake braking -> for lane change behavior
            for(auto it = ego_->statesHistory.begin(); it != ego_->statesHistory.end() - 1; it++){
                if(it->a == ego_->rssP.aMinBrake &&
                   std::next(it)->a == ego_->rssP.aMinBrake){
                    return true;
                }
            }
        }
        return false;
    }

    double egoUtility() const{
        if(egoId < 0){
            return 0.;
        }
        double uSum = 0.;
        for(const auto& s: ego_->statesHistory){
            // minimum 0.9 utility when v > vd
            uSum = uSum + (s.v <= ego_->desiredV ? s.v / ego_->desiredV: std::max(1. - (s.v / ego_->desiredV - 1.), 0.9));
        }
        uSum = uSum / double(ego_->statesHistory.size());
        // cost for not holding desired thw
        double thwSum = 0.;
        if(!ego_->thwHistory.empty()){
            for(const auto& thwRatio: ego_->thwHistory){
                // cost only when thw < 0.9 * thw desired
                if(thwRatio < 0.9 && thwRatio > 0){
                    thwSum = thwSum + thwRatio - 0.9;
                }
            }
            thwSum = thwSum / double(ego_->thwHistory.size());
        }
        // std::cout << "u sum: " << uSum << " thw sum: " << thwSum << std::endl;
//        if(uSum + thwSum < -1000){
//            for(const auto& s: ego_->statesHistory){
//                std::cout <<" v " << s.v << std::endl;
//            }
//            for(const auto& s: ego_->thwHistory){
//                std::cout <<" thw ratio " << s << std::endl;
//            }
//        }
        return uSum + thwSum;
    }

    double egoComfort() const{
        // use the worst 1/5 acc -> focus on worst acceleration, acceleration should be half of penalty of deceleration
        if(egoId < 0){
            return 0.;
        }
        double cSum = 0.;
        std::vector<double> accList;
        for(const auto& s: ego_->statesHistory){
            if(s.a > 0){
                accList.push_back(0.5 * std::abs(s.a));
            }else{
                accList.push_back(std::abs(s.a));
            }
        }
        std::sort(accList.begin(), accList.end(), std::greater<>());
        for(int i = 0; i < int(0.2 * accList.size()); ++i){
//            std::cout << accList[i] << " " << ego_->rssP.aMinBrake << std::endl;
            cSum = cSum + (1 + accList[i] / ego_->rssP.aMinBrake);
        }
        return cSum / double(int(0.2 * accList.size()));
    }

    std::pair<double, double> meanComfortUtilityOthers() const{
        //take the minimum utility and comfort of vehicles, worst influence on others
        std::vector<double> sumComfortObjs;
        std::vector<double> sumUtilityObjs;
        int numObjects = 0;
        for(const auto& obj: agents){
            if(!obj.second->isEgo){
                double sumAObj = std::accumulate(obj.second->statesHistory.begin(), obj.second->statesHistory.end(), 0.,
                                                 [](double sum, const State& s) { return sum + std::abs(s.a); });
                double sumVObj = std::accumulate(obj.second->statesHistory.begin(), obj.second->statesHistory.end(), 0.,
                                                 [](double sum, const State& s) { return sum + s.v; });
                double meanAObj = sumAObj / double(obj.second->statesHistory.size());
                double meanVObj = sumVObj / double(obj.second->statesHistory.size());
                double comfortObj = 1. + meanAObj / obj.second->rssP.aMinBrake;
                double utilityObj = meanVObj >= obj.second->desiredV ? 1.0 : meanVObj / obj.second->desiredV;
                sumUtilityObjs.push_back(utilityObj);
                sumComfortObjs.push_back(comfortObj);
                numObjects++;
            }
        }
        if(numObjects > 0){
            return {*std::min_element(sumComfortObjs.begin(), sumComfortObjs.end()),
                    *std::min_element(sumUtilityObjs.begin(), sumUtilityObjs.end())};
        }
        return {1., 1.};
    }

    void matchAgentsOnCorridor(){
        bool hasLaneChangeOrFirstMatch = false;
        for(auto& a: agents){
            // check if first time match
            if(!a.second->corridorIdHistory.empty()){
                // if encounter lane changes, rematch agents and resort on each lane
                if(!map.matched(a.second->corridorIdHistory.back(), a.second->x, a.second->y)){
                    hasLaneChangeOrFirstMatch = true;
                    const auto matchedCorridor = map.matchedCorridor(a.second->x, a.second->y);
                    if(matchedCorridor){
                        // assign new corridor
                        a.second->corridorIdHistory.push_back(matchedCorridor->id);
                        if(matchedCorridor->type == "main" && a.second->behaviorModel == "merging"){
                            a.second->setBehaviorModel("idm_lc");
                        }
                        if(matchedCorridor->type == "exit" && a.second->behaviorModel == "exit"){
                            a.second->setBehaviorModel("idm");
                        }
                    }
                }
            } else {// first time match
                hasLaneChangeOrFirstMatch = true;
                const auto matchedCorridor = map.matchedCorridor(a.second->x, a.second->y);
                if(matchedCorridor){
                    // assign new corridor
                    a.second->corridorIdHistory.push_back(matchedCorridor->id);
                    if(matchedCorridor->type == "main" && a.second->behaviorModel == "merging"){
                        a.second->setBehaviorModel("idm_lc");
                    }
                    if(matchedCorridor->type == "exit" && a.second->behaviorModel == "exit"){
                        a.second->setBehaviorModel("idm");
                    }
                    a.second->setMoveBasedLaneChangeP(moveBasedLaneChangeProbability(
                            matchedCorridor->signedDistanceToCenter(a.second->y), a.second->vy));
                }
            }
            // check if ego reached target
            if(a.second->isEgo){
                ego_ = a.second;
                if(!ego_->targetReached){
                    ego_->targetReached = ego_->corridorIdHistory.back() == ego_->targetLaneId;
                }
            }
        }
        if(hasLaneChangeOrFirstMatch){
            sortAgentsOnCorridors();
        }
        // check if targetGapId is still on target lane
        if(ego_ && ego_->targetGapId >= 0){
            auto previousTarget = agents.find(ego_->targetGapId)->second;
            if(previousTarget->corridorIdHistory.back() != ego_->targetLaneId){
                auto& agentsOnTarget = sortedAgentsOnCorridor.find(ego_->targetLaneId)->second;
                for(auto it = agentsOnTarget.rbegin(); it != agentsOnTarget.rend(); ++it){
                    if((*it)->x < previousTarget->x){
                        ego_->targetGapId = (*it)->id;
                        break;
                    }
                }
                if(ego_->targetGapId == previousTarget->id){
                    ego_->targetGapId = -1;
                }
            }
        }
    }

    void sortAgentsOnCorridors(){
        sortedAgentsOnCorridor.clear();
        for(const auto& c: map.corridors){
            sortedAgentsOnCorridor.insert({c.first, {}});
        }
        for(const auto& a: agents){
            if(!a.second->corridorIdHistory.empty()){
                sortedAgentsOnCorridor.find(a.second->corridorIdHistory.back())->second.push_back(a.second);
            }
        }
        for(const auto& c: map.corridors){
            auto& agentsOnCorridor = sortedAgentsOnCorridor.find(c.first)->second;
            std::sort(agentsOnCorridor.begin(), agentsOnCorridor.end(),
                      [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x < b->x;});
        }
    }

    AgentPointer frontAgent(const AgentPointer& a) const{
        if(a->corridorIdHistory.empty()){
            return nullptr;
        }
        const auto sortedAgentsOnEgoLane = sortedAgentsOnCorridor.find(a->corridorIdHistory.back())->second;
        const auto front = std::upper_bound(sortedAgentsOnEgoLane.begin(), sortedAgentsOnEgoLane.end(), a,
                                            [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x < b->x;});
        if(front != sortedAgentsOnEgoLane.end()){
            return *front;
        }
        return nullptr;
    }

    AgentPointer followingAgent(const AgentPointer& a) const{
        if(a->corridorIdHistory.empty()){
            return nullptr;
        }
        const auto sortedAgentsOnEgoLane = sortedAgentsOnCorridor.find(a->corridorIdHistory.back())->second;
        const auto follow = std::upper_bound(sortedAgentsOnEgoLane.rbegin(), sortedAgentsOnEgoLane.rend(), a,
                                             [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x > b->x;});
        if(follow != sortedAgentsOnEgoLane.rend()){
            return *follow;
        }
        return nullptr;
    }

    Neighbor neighborAgents(const AgentPointer& ego,
                            bool includeLeftLane=true,
                            bool includeRightLane=true,
                            bool includeEgoLane=true) const{
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
        if(ego->corridorIdHistory.empty()){
            return {lFF, lF, lB, lBB, f, b, rFF, rF, rB, rBB};
        }
        const auto corridorId = ego->corridorIdHistory.back();
        const auto lCorridorId = map.corridors.find(corridorId)->second.lId;
        const auto rCorridorId = map.corridors.find(corridorId)->second.rId;
        if(lCorridorId and includeLeftLane){
            const auto lAgents = sortedAgentsOnCorridor.find(lCorridorId.value())->second;
            const auto front = std::upper_bound(lAgents.begin(), lAgents.end(), ego,
                                                [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x <= b->x;});
            if(front != lAgents.end()){
                lF = *front;
                const auto ff = std::next(front);
                if(ff != lAgents.end()){
                    lFF = *ff;
                }
            }
            const auto follow = std::upper_bound(lAgents.rbegin(), lAgents.rend(), ego,
                                                 [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x > b->x;});
            if(follow != lAgents.rend()){
                lB = *follow;
                const auto bb = std::next(follow);
                if(bb != lAgents.rend()){
                    lBB = *bb;
                }
            }
        }
        if(rCorridorId and includeRightLane){
            const auto rAgents = sortedAgentsOnCorridor.find(rCorridorId.value())->second;
            const auto front = std::upper_bound(rAgents.begin(), rAgents.end(), ego,
                                                [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x <= b->x;});
            if(front != rAgents.end()){
                rF = *front;
                const auto ff = std::next(front);
                if(ff != rAgents.end()){
                    rFF = *ff;
                }
            }
            const auto follow = std::upper_bound(rAgents.rbegin(), rAgents.rend(), ego,
                                                 [](const AgentPointer& a, const AgentPointer& b)->bool{ return a->x > b->x;});
            if(follow != rAgents.rend()){
                rB = *follow;
                const auto bb = std::next(follow);
                if(bb != rAgents.rend()){
                    rBB = *bb;
                }
            }
        }
        if(includeEgoLane){
            f = frontAgent(ego);
            b = followingAgent(ego);
        }
        return {lFF, lF, lB, lBB, f, b, rFF, rF, rB, rBB};
    }

    void step(const double& dt){
        std::vector<Action> actions;
        for(auto& a: agents){
            if(a.second->isEgo){
                if(a.second->behaviorModel == "idm_lc" || a.second->behaviorModel == "idm"){
                    if(a.second->direction == "left" || a.second->direction == "right"){
                        if (!a.second->targetReached) {
                            actions.push_back(customizedLaneChangeBehavior(*this, a.second));
                            continue;
                        } else {
                            actions.push_back(idmBehaviorAndYieldingToMergingAgents(*this, a.second, false));
                            continue;
                        }
                    }else{
                        actions.push_back(customizedLaneChangeBehavior(*this, a.second));
                        continue;
                    }
                }else if(a.second->behaviorModel == "merging" or a.second->behaviorModel == "exit") {
                    if (!a.second->targetReached) {
                        actions.push_back(customizedMergingBehavior(*this, a.second));
                        continue;
                    } else {
                        actions.push_back(idmBehaviorAndYieldingToMergingAgents(*this, a.second, false));
                        continue;
                    }
                }
            }
            if(a.second->behaviorModel == "idm_lc"){
                actions.push_back(laneChangeBehaviorMOBIL(*this, a.second));
            }else if (a.second->behaviorModel == "idm") {
                actions.push_back(idmBehaviorAndYieldingToMergingAgents(*this, a.second, true));
            }else if (a.second->behaviorModel == "merging") {
                actions.push_back(mergingBehaviorClosestGap(*this, a.second));
            }
        }
        int i = 0;
        for(auto& a: agents){
            a.second->step(actions[i++], dt);
        }
        matchAgentsOnCorridor();
        step_++;
//        std::cout << "plan: " << std::chrono::duration<double, std::milli>(t22 - t11).count() << std::endl;
//        std::cout << "step: " << std::chrono::duration<double, std::milli>(t33 - t22).count() << std::endl;
//        std::cout << "match: " << std::chrono::duration<double, std::milli>(t44 - t33).count() << std::endl;
    }

    int step_{0};
    int egoId{-1};
    int egoTargetGapId{-2};
    std::string egoDirection{"keep_lane"};
    MapMultiLane map;
    AgentPointer ego_;
    std::unordered_map<int, AgentPointer> agents;
    std::unordered_map<int, AgentsPointer> sortedAgentsOnCorridor;  // int: corridor id, agents: sorted with x increasing
    std::unordered_map<int, Agents> cachedSortedAgentsOnCorridor;
};

}
