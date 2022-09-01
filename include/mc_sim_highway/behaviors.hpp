#include "environment.hpp"

namespace mc_sim_highway{

inline Action idmBehavior(const Environment& env, AgentPointer& agent, bool probabilistic = false) {
        double ax;
        double vy = 0.;
        if(agent->corridorIdHistory.empty()){
            return Action{0., 0.};
        }
        const auto corridor = env.map.corridors.find(agent->corridorIdHistory.back())->second;
        AgentsPointer all;
        const auto frontAgent = env.frontAgent(agent);
        all.push_back(frontAgent);
        if(frontAgent){
            const double thw = (frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l)) / agent->v;
            agent->thwHistory.push_back(thw / agent->idmP.thw);
        }else{
            agent->thwHistory.push_back(1.0);
        }

        double desiredV = agent->desiredV;
        if(agent->isEgo){
            desiredV = std::min(1.1 * corridor.vLimit, agent->desiredV);
//        std::cout << "agent " << agent->id << " vd: " << agent->desiredV << " v limit" << corridor.vLimit << std::endl;
        }
        ax = idmAccFrontBack(agent->idmP, all, nullptr, agent->x, agent->v, agent->l, desiredV);
        const double signedDisToCenter = corridor.signedDistanceToCenter(agent->y);
        const double vyAbs = std::min(std::max(std::abs(signedDisToCenter), 0.1), 1.3);
        vy = -signedDisToCenter / (std::abs(signedDisToCenter) + 1e-10) * vyAbs;

        if(probabilistic){
            ax = ax + ((double) rand() / (RAND_MAX) - 0.5);
        }
        if(frontAgent && frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l) <
                         rssDis(frontAgent->v, agent->v, agent->rssP.rTimeEgo, agent->rssP.aMinBrake, agent->rssP.aMaxBrake)){
            ax = agent->rssP.aMinBrake;
        }
        return Action{ax, vy};
}

inline Action idmBehaviorAndYieldingToMergingAgents(const Environment& env,
                                                    AgentPointer& agent,
                                                    bool probabilistic = false) {
    double ax;
    double vy = 0.;
    if(agent->corridorIdHistory.empty()){
        return Action{0., 0.};
    }
    const auto corridor = env.map.corridors.find(agent->corridorIdHistory.back())->second;
    const auto frontAgent = env.frontAgent(agent);
    if(frontAgent){
        const double thw = (frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l)) / agent->v;
        agent->thwHistory.push_back(thw / agent->idmP.thw);
    }else{
        agent->thwHistory.push_back(1.0);
    }

    double desiredV = agent->desiredV;
    if(agent->isEgo){
        desiredV = std::min(1.1 * corridor.vLimit, agent->desiredV);
//        std::cout << "agent " << agent->id << " vd: " << agent->desiredV << " v limit" << corridor.vLimit << std::endl;
    }
    const double axOnlyFront = idmAccFrontBack(agent->idmP, {frontAgent}, nullptr, agent->x, agent->v, agent->l, desiredV);

    const double signedDisToCenter = corridor.signedDistanceToCenter(agent->y);
    const double vyAbs = std::min(std::max(std::abs(signedDisToCenter), 0.1), 1.3);
    vy = -signedDisToCenter / (std::abs(signedDisToCenter) + 1e-10) * vyAbs;
    // yielding behavior to merging vehicle
    // if decide to yield, choose to decelerate or do lane change based on Mobil
    AgentsPointer mergingAndExitVehicles;
    AgentsPointer yieldingVehicles;
    if(corridor.rId && env.map.corridors.find(*(corridor.rId))->second.type == "merging"){
        const AgentsPointer mergingVehicles = env.neighborAgents(agent, false, true, false).right();
        mergingAndExitVehicles.insert(mergingAndExitVehicles.end(), mergingVehicles.begin(), mergingVehicles.end());
    }
    if(env.map.hasExitLane() && corridor.lId && env.map.corridors.find(*(corridor.lId))->second.type == "main"){
        for(const auto& a: env.agents){
            if(a.second->corridorIdHistory.back() == corridor.lId && a.second->behaviorModel == "exit"){
                mergingAndExitVehicles.push_back(a.second);
            }
        }
    }
    for(const auto& vehicle: mergingAndExitVehicles){
        const double newP = agent->yieldingModel.yieldingProbability(vehicle->x - agent->x, vehicle->l, vehicle->v,
                                                                     agent->l, agent->v, agent->statesHistory[0].a);
        agent->yieldingIntentionToOthers.insert({vehicle->id, YieldingIntention(vehicle->id, newP)});
        agent->yieldingIntentionToOthers.find(vehicle->id)->second.updateIntention(newP);
        if(agent->yieldingIntentionToOthers.find(vehicle->id)->second.yielding){
            yieldingVehicles.push_back(vehicle);
        }
    }
    const double axYielding = idmAccFrontBack(agent->idmPYielding, yieldingVehicles,
                                              nullptr, agent->x, agent->v, agent->l, desiredV);

    ax = std::min(axOnlyFront, axYielding);
    if(probabilistic){
        ax = ax + ((double) rand() / (RAND_MAX) - 0.5);
    }
    if(frontAgent && frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l) <
                     rssDis(frontAgent->v, agent->v, agent->rssP.rTimeEgo, agent->rssP.aMinBrake, agent->rssP.aMaxBrake)){
        ax = agent->rssP.aMinBrake;
    }
    return Action{ax, vy};
}

inline Action laneChangeBehaviorMOBIL(const Environment& env, AgentPointer& agent, const Action& actionIdm,
                                      std::string& commitToDecision, int& commitKeepLaneStep, int& targetLaneId,
                                      bool isMCSAgent,
                                      bool safetyRequired){

    int keepDecisionStep = 10;
    if(agent->commitToDecision == "keep_lane" && agent->commitKeepLaneStep < keepDecisionStep){
        // reevaluate if keep lane change after roughly 1 second
        agent->commitKeepLaneStep++;
        commitToDecision = agent->commitToDecision;
        commitKeepLaneStep = agent->commitKeepLaneStep;
        targetLaneId = agent->targetLaneId;
        return actionIdm;
    }
    const auto corridor = env.map.corridors.find(agent->corridorIdHistory.back())->second;
    bool lCorridorValid = corridor.lId && env.map.corridors.find(*(corridor.lId))->second.type == "main";
    bool rCorridorValid = corridor.rId && env.map.corridors.find(*(corridor.rId))->second.type == "main";
    if(!lCorridorValid && !rCorridorValid){
        commitToDecision = agent->commitToDecision;
        commitKeepLaneStep = agent->commitKeepLaneStep;
        targetLaneId = agent->targetLaneId;
        return actionIdm;
    }
    const double pFactor = agent->yieldingModel.param.politenessFactor;
    const double deltaA = agent->yieldingModel.param.deltaA;  // changing lane threshold
    const double biasA = agent->yieldingModel.param.biasA;  // bias for asymmetric europe lane change rule
    double lAccGain = -1e10;  // total acc gain - deltaA - biasA
    double rAccGain = -1e10;  // total acc gain - deltaA + biasA
    Action changeLeftAction = actionIdm;
    Action changeRightAction = actionIdm;
    Corridor lC = corridor;
    Corridor rC = corridor;
    bool rssSafeL = false;
    bool rssSafeR = false;
    bool rssSafeLRelax = false;
    bool rssSafeRRelax = false;
    // compute acc gain from old follower
    double aOldFollower = 0.;
    double aOldFollowerAfter = 0.;
    const AgentPointer oldFollower = env.followingAgent(agent);
    const AgentPointer oldFront = env.frontAgent(agent);
    if(oldFollower){
        aOldFollower = idmAccFrontBack(oldFollower->idmP, {oldFront, agent}, nullptr, oldFollower->x,
                                       oldFollower->v, oldFollower->l, oldFollower->desiredV);
        aOldFollowerAfter = idmAccFrontBack(oldFollower->idmP, {oldFront}, nullptr, oldFollower->x,
                                            oldFollower->v, oldFollower->l, oldFollower->desiredV);
    }
    const double accGainOldFollower = aOldFollowerAfter - aOldFollower;
    if(lCorridorValid){
        lC = env.map.corridors.find(*corridor.lId)->second;
        const AgentsPointer possibleLeft = env.neighborAgents(agent, true, false, false).possibleLeft();
        const AgentPointer leftF = possibleLeft[2];
        const AgentPointer leftB = possibleLeft[1];
        double desiredV = std::min(1.1 * lC.vLimit, agent->desiredV);
        const double aEgoNewL = idmAccFrontBack(agent->idmP, {leftF}, leftB, agent->x, agent->v, agent->l, desiredV);
        std::optional<double> fd = std::nullopt;
        std::optional<double> fv = std::nullopt;
        std::optional<double> bd = std::nullopt;
        std::optional<double> bv = std::nullopt;
        if(leftF){
            fd = leftF->x - agent->x - 0.5 * (leftF->l + agent->l);
            fv = leftF->v;
        }
        if(leftB){
            bd = leftB->x - agent->x + 0.5 * (leftB->l + agent->l);
            bv = leftB->v;
        }
        rssSafeL = rssSafe(fd, fv, bd, bv, agent->v, agent->rssP);
        rssSafeLRelax = rssSafe(fd, fv, bd, bv, agent->v, agent->rssPUnsafe);
        changeLeftAction.ax = aEgoNewL;
        if(actionIdm.ax == agent->rssP.aMinBrake){
            changeLeftAction.ax = actionIdm.ax;
        }
        changeLeftAction.vy = std::min(0.17 * agent->v, 1.0);

        // compute acc gain
        const double accGainEgo = aEgoNewL - actionIdm.ax;
        double aNewFollower = 0.;
        double aNewFollowerAfter = 0.;
        if(leftB){
            aNewFollower = idmAccFrontBack(leftB->idmP, {leftF}, nullptr, leftB->x, leftB->v, leftB->l, leftB->desiredV);
            aNewFollowerAfter = idmAccFrontBack(leftB->idmP, {leftF, agent}, nullptr, leftB->x, leftB->v, leftB->l, leftB->desiredV);
        }
        const double accGainNewFollower = aNewFollowerAfter - aNewFollower;
        lAccGain = accGainEgo + pFactor * (accGainNewFollower + accGainOldFollower) - deltaA - biasA;
    }
    if(rCorridorValid){
        rC = env.map.corridors.find(*corridor.rId)->second;
        const AgentsPointer possibleRight = env.neighborAgents(agent, false, true, false).possibleRight();
        const AgentPointer rightF = possibleRight[2];
        const AgentPointer rightB = possibleRight[1];
        double desiredV = std::min(1.1 * rC.vLimit, agent->desiredV);
        const double aEgoNewR = idmAccFrontBack(agent->idmP, {rightF}, rightB, agent->x, agent->v, agent->l, desiredV);
        std::optional<double> fd = std::nullopt;
        std::optional<double> fv = std::nullopt;
        std::optional<double> bd = std::nullopt;
        std::optional<double> bv = std::nullopt;
        if(rightF){
            fd = rightF->x - agent->x - 0.5 * (rightF->l + agent->l);
            fv = rightF->v;
        }
        if(rightB){
            bd = rightB->x - agent->x + 0.5 * (rightB->l + agent->l);
            bv = rightB->v;
        }
        rssSafeR = rssSafe(fd, fv, bd, bv, agent->v, agent->rssP);
        rssSafeRRelax = rssSafe(fd, fv, bd, bv, agent->v, agent->rssPUnsafe);
        changeRightAction.ax = aEgoNewR;
        if(actionIdm.ax == agent->rssP.aMinBrake){
            changeLeftAction.ax = actionIdm.ax;
        }
        changeRightAction.vy = std::max(-0.17 * agent->v, -1.0);

        // compute acc gain
        const double accGainEgo = aEgoNewR - actionIdm.ax;
        double aNewFollower = 0.;
        double aNewFollowerAfter = 0.;
        if(rightB){
            aNewFollower = idmAccFrontBack(rightB->idmP, {rightF}, nullptr, rightB->x, rightB->v, rightB->l, rightB->desiredV);
            aNewFollowerAfter = idmAccFrontBack(rightB->idmP, {agent, rightF}, nullptr, rightB->x, rightB->v, rightB->l, rightB->desiredV);
        }
        const double accGainNewFollower = aNewFollowerAfter - aNewFollower;
        rAccGain = accGainEgo + pFactor * (accGainNewFollower + accGainOldFollower) - deltaA + biasA;
    }
    if(agent->commitToDecision == "left" && agent->targetLaneId != corridor.id && rssSafeLRelax){
        // uncommit when lane change finished
        commitToDecision = agent->commitToDecision;
        commitKeepLaneStep = agent->commitKeepLaneStep;
        targetLaneId = agent->targetLaneId;
        return changeLeftAction;
    }
    if(agent->commitToDecision == "right" && agent->targetLaneId != corridor.id && rssSafeRRelax){
        commitToDecision = agent->commitToDecision;
        commitKeepLaneStep = agent->commitKeepLaneStep;
        targetLaneId = agent->targetLaneId;
        return changeRightAction;
    }
    // do lane change according to softmax probability and commit
    agent->commitKeepLaneStep = 0;
    const double qKeepLane = 0.;
    double qLeft = lAccGain;
    double qRight = rAccGain;
    const double sumE = std::exp(qKeepLane) + std::exp(qLeft) + std::exp(qRight);
    const double pLeft = std::exp(qLeft) / sumE;
    const double pRight = std::exp(qRight) / sumE;
    LaneChangeProbability lcProbability = {pLeft, 1 - pLeft - pRight, pRight};
    if(isMCSAgent){
        // first step, consider motion in lc probability estimation
        if(agent->commitToDecision == "not_decided" && agent->statesHistory.size() <= 1){
            if(agent->aggressiveAgent){
                lcProbability = laneChangeProbability(lcProbability, agent->moveBasedLCP, rssSafeLRelax, rssSafeRRelax);
            }else{
                lcProbability = laneChangeProbability(lcProbability, agent->moveBasedLCP,
                                                      rssSafeL || (agent->moveBasedLCP.left >= 0.5 && rssSafeLRelax),
                                                      rssSafeR || (agent->moveBasedLCP.right >= 0.5 && rssSafeRRelax));
            }
        }else{
            if(agent->aggressiveAgent){
                lcProbability = laneChangeProbability(lcProbability, {0., 0., 0.}, rssSafeLRelax, rssSafeRRelax);
            }else{
                lcProbability = laneChangeProbability(lcProbability, {0., 0., 0.}, rssSafeL, rssSafeR);
            }
        }

    }else{
        if(!safetyRequired){
            lcProbability = laneChangeProbability(lcProbability, {0., 0., 0.}, rssSafeLRelax, rssSafeRRelax);
        }else{
            lcProbability = laneChangeProbability(lcProbability, {0., 0., 0.}, rssSafeL, rssSafeR);
        }
    }
    // trucks have less lane change and do not change to most left lane
    if(agent->l > 6){
        lcProbability = truckLaneChangeProbability(lcProbability, corridor.lId, env.map.noTruckLane);
    }
    const double randValue = (double)rand() / RAND_MAX;
//    std::cout << "Model based Probability " << pLeft << " " << 1 - pLeft - pRight << " " << pRight << std::endl;
//    std::cout << "Move based Probability " << agent->moveBasedLCP.left << " " << agent->moveBasedLCP.keep << " " << agent->moveBasedLCP.right << std::endl;
//    std::cout << "Lc Probability " << lcProbability.left << " " << lcProbability.keep << " " << lcProbability.right << std::endl;
//    std::cout << "step: " << agent->statesHistory.size() << std::endl;
    if(randValue < lcProbability.left && lCorridorValid){
        agent->commitToDecision = "left";
        agent->targetLaneId = lC.id;
        commitToDecision = agent->commitToDecision;
        commitKeepLaneStep = agent->commitKeepLaneStep;
        targetLaneId = agent->targetLaneId;
        return changeLeftAction;
    }else if(randValue < lcProbability.left + lcProbability.right && rCorridorValid){
        agent->commitToDecision = "right";
        agent->targetLaneId = rC.id;
        commitToDecision = agent->commitToDecision;
        commitKeepLaneStep = agent->commitKeepLaneStep;
        targetLaneId = agent->targetLaneId;
        return changeRightAction;
    }
    agent->commitToDecision = "keep_lane";
    agent->targetLaneId = -1;
    commitToDecision = agent->commitToDecision;
    commitKeepLaneStep = agent->commitKeepLaneStep;
    targetLaneId = agent->targetLaneId;
    return actionIdm;
}
    // after reaching target lane, at least keep lane for 5 steps
//    if(agent->commitToDecision == "left" || agent->commitToDecision == "right"){
//        agent->commitToDecision = "keep_lane";
//        agent->targetLaneId = -1;
//        agent->commitKeepLaneStep = 0;
//        commitToDecision = agent->commitToDecision;
//        commitKeepLaneStep = agent->commitKeepLaneStep;
//        targetLaneId = agent->targetLaneId;
//        return actionIdm;
//    }
//    if(isMCSAgent){
//        keepDecisionStep = agent->aggressiveAgent ? 10: 5;
//    }else{
//        keepDecisionStep = safetyRequired ? 5: 10;
//    }

//    if(isMCSAgent){
//        if(agent->aggressiveAgent){
//            qLeft = std::max(lAccGain, -3.);
//            qRight = std::max(rAccGain, -3.);
//        }else{
//            qLeft = rssSafeL ? lAccGain : -100;
//            qRight = rssSafeR ? rAccGain : -100;
//        }
//    }else{
//        if(!safetyRequired){
//            qLeft = std::max(lAccGain, -3.);
//            qRight = std::max(rAccGain, -3.);
//        }else{
//            qLeft = rssSafeL ? lAccGain : -100;
//            qRight = rssSafeR ? rAccGain : -100;
//        }
//    }

inline Action laneChangeBehaviorMOBIL(const Environment& env, AgentPointer& agent){
    std::string flagPlaceholder = "not_decided";
    int keepLaneStepPlaceholder = 0;
    int targetLaneIdPlaceholder = 0;
    auto actionIdm = idmBehaviorAndYieldingToMergingAgents(env, agent, true);
    return laneChangeBehaviorMOBIL(env, agent, actionIdm, flagPlaceholder, keepLaneStepPlaceholder,
                                   targetLaneIdPlaceholder, true, false);
}

inline Action customizedLaneChangeBehavior(const Environment& env, AgentPointer& agent, const std::string& direction){
    double ax;
    double vy = 0.;
    if(agent->corridorIdHistory.empty()){
        return Action{0., 0.};
    }
    const auto corridor = env.map.corridors.find(agent->corridorIdHistory.back())->second;
    const auto frontAgent = env.frontAgent(agent);
    if(frontAgent){
        const double thw = (frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l)) / agent->v;
        agent->thwHistory.push_back(thw / agent->idmP.thw);
    }else{
        agent->thwHistory.push_back(1.0);
    }
    double desiredV = std::min(1.1 * corridor.vLimit, agent->desiredV);
    if(direction == "keep_lane"){
        ax = idmAccFrontBack(agent->idmP, {frontAgent}, nullptr, agent->x, agent->v, agent->l, desiredV);
    }
    if(direction == "dcc"){
        auto newIdmP = agent->idmP;
        newIdmP.thw = 2.0 * agent->idmP.thw;
        ax = idmAccFrontBack(newIdmP, {frontAgent}, nullptr, agent->x, agent->v, agent->l, 0.8 * desiredV);
    }
    if(direction == "acc"){
        auto newIdmP = agent->idmP;
        newIdmP.thw = 0.9;  // minimum thw
        ax = idmAccFrontBack(newIdmP, {frontAgent}, nullptr, agent->x, agent->v, agent->l, 1.1 * desiredV);
    }
    const double signedDisToCenter = corridor.signedDistanceToCenter(agent->y);
    const double vyAbs = std::min(std::max(std::abs(signedDisToCenter), 0.1), 1.3);
    vy = -signedDisToCenter / (std::abs(signedDisToCenter) + 1e-10) * vyAbs;
    if(frontAgent && frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l) <
                     rssDis(frontAgent->v, agent->v, agent->rssP.rTimeEgo, agent->rssP.aMinBrake, agent->rssP.aMaxBrake)){
        ax = agent->rssP.aMinBrake;
    }
    Action actionIdm = {ax, vy};

    if(direction == "keep_lane" || direction == "acc" || direction == "dcc"){
        // add thw history for additionally compute utility cost when not holding desired thw
        return actionIdm;
    }
    AgentsPointer possibleNeighbor;
    double vySign = 1.;
    if(direction == "left"){
        const auto targetCorridorId = corridor.lId;
        if(!targetCorridorId){ return actionIdm; }
        const auto targetCorridor = env.map.corridors.find(*(corridor.lId));
        if(targetCorridor->second.type != "main"){ return actionIdm; }
        possibleNeighbor = env.neighborAgents(agent, true, false, false).possibleLeft();
    }else{
        const auto targetCorridorId = corridor.rId;
        if(!targetCorridorId){ return actionIdm; }
        const auto targetCorridor = env.map.corridors.find(*(corridor.rId));
        if(targetCorridor->second.type != "main"){ return actionIdm; }
        possibleNeighbor = env.neighborAgents(agent, false, true, false).possibleRight();
        vySign = -1.;
    }
    const AgentPointer f = possibleNeighbor[2];
    const AgentPointer b = possibleNeighbor[1];
    ax = idmAccFrontBack(agent->idmP, {f, frontAgent}, b, agent->x, agent->v, agent->l, desiredV);
    if(actionIdm.ax == agent->rssP.aMinBrake){
        ax = actionIdm.ax;
    }
    std::optional<double> fd = std::nullopt;
    std::optional<double> fv = std::nullopt;
    std::optional<double> bd = std::nullopt;
    std::optional<double> bv = std::nullopt;
    if(f){
        fd = f->x - agent->x - 0.5 * (f->l + agent->l);
        fv = f->v;
    }
    if(b){
        bd = b->x - agent->x + 0.5 * (b->l + agent->l);
        bv = b->v;
    }
    bool laneChangeSafe = rssSafe(fd, fv, bd, bv, agent->v, agent->rssP);
    vy = vySign * std::min(0.17 * agent->v, 1.0);
    if(!laneChangeSafe){
        vy = actionIdm.vy;
    }
    return Action{ax, vy};
}

inline Action customizedLaneChangeBehavior(const Environment& env, AgentPointer& agent){
    return customizedLaneChangeBehavior(env, agent, agent->direction);
}

inline Action customizedMergingBehavior(const Environment& env, AgentPointer& agent,
                                        const std::string& direction, const int& targetGapId){
//    std::cout << direction << " " << targetGapId << std::endl;
    double ax = 0.;
    double vy = 0.;
    if(agent->corridorIdHistory.empty()){
        std::cout << "CustomizedMergingBehavior: Vehicle with id " << agent->id << " is not matched!" << std::endl;
        return Action{0., 0.};
    }
    const auto corridor = env.map.corridors.find(agent->corridorIdHistory.back())->second;
    const auto frontAgent = env.frontAgent(agent);
    if(frontAgent){
        const double thw = (frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l)) / agent->v;
        agent->thwHistory.push_back(thw / agent->idmP.thw);
    }else{
        agent->thwHistory.push_back(1.0);
    }
    double vySign = 1.;
    std::optional<int> targetCorridorId;
    if(direction == "left"){
        targetCorridorId = corridor.lId;
        if(!targetCorridorId){ std::cout << "CustomizedMergingBehavior: No corridor at left side!" << std::endl; }
    }else{
        targetCorridorId = corridor.rId;
        if(!targetCorridorId){ std::cout << "CustomizedMergingBehavior: No corridor at right side!" << std::endl; }
        vySign = -1.;
    }
    AgentPointer ff = nullptr;
    AgentPointer f = nullptr;
    AgentPointer b = nullptr;
    if(targetGapId == -1){
        auto neighbor = direction == "left" ? env.neighborAgents(agent, true, false, false).left():
                env.neighborAgents(agent, false, true, false).right();
        if(neighbor.size() > 0){ f = neighbor[0]; }
        if(neighbor.size() > 1){ ff = neighbor[1]; }
    }else {
        auto targetVehicle = env.agents.find(targetGapId);
        if(targetVehicle == env.agents.end()){
            std::cout << "CustomizedMergingBehavior: no gap id " << targetGapId << " -> go to last gap" << std::endl;
            auto neighbor = direction == "left" ? env.neighborAgents(agent, true, false, false).left():
                            env.neighborAgents(agent, false, true, false).right();
            if(neighbor.size() > 0){ f = neighbor[0]; }
            if(neighbor.size() > 1){ ff = neighbor[1]; }
        }else{
            b = env.agents.find(targetGapId)->second;
            f = env.frontAgent(b);
            if(f){ ff = env.frontAgent(f); }
        }
    }
    std::optional<double> ffd = std::nullopt;
    std::optional<double> ffv = std::nullopt;
    std::optional<double> fd = std::nullopt;
    std::optional<double> fv = std::nullopt;
    std::optional<double> bd = std::nullopt;
    std::optional<double> bv = std::nullopt;
    if(f){
        fd = f->x - agent->x - 0.5 * (f->l + agent->l);
        fv = f->v;
    }
    if(ff){
        ffd = ff->x - agent->x - 0.5 * (ff->l + agent->l);
        ffv = ff->v;
    }
    if(b){
        bd = b->x - agent->x + 0.5 * (b->l + agent->l);
        bv = b->v;
    }
    double desiredV = std::min(1.1 * corridor.vLimit, agent->desiredV);
    bool laneChangeSafe = rssSafeRelax(ffd, ffv, fd, fv, bd, bv, agent->v, desiredV, agent->rssP, agent->idmP);
    // reach beginning of target lane
    const auto targetCorridor = env.map.corridors.find(*targetCorridorId)->second;
    bool reachCorridorBegin = targetCorridor.reachCorridorBegin(agent->x);
    ax = idmAccFrontBack(agent->idmP, {f, ff, frontAgent}, b, agent->x, agent->v, agent->l, desiredV);
    // safety check
    bool rssSafeFront = true;
    bool fallBackExists = corridor.fallBackExist(agent->x, agent->v, agent->l, agent->rssP.aMinBrake, agent->rssP.rTimeEgo);
    if(frontAgent && frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l) <
                     rssDis(frontAgent->v, agent->v, agent->rssP.rTimeEgo, agent->rssP.aMinBrake, agent->rssP.aMaxBrake)){
        rssSafeFront = false;
    }
    if(!rssSafeFront || (!fallBackExists && !laneChangeSafe)){
        ax = agent->rssP.aMinBrake;
    }
    const double signedDisToCorridorBorder = direction == "left" ?
            corridor.signedDistanceToLeftBorder(agent->y, agent->d) :
            corridor.signedDistanceToRightBorder(agent->y, agent->d);
    bool farFromBorder = direction == "left" ? signedDisToCorridorBorder < -0.5: signedDisToCorridorBorder > 0.5;
    if(farFromBorder || (laneChangeSafe && rssSafeFront && reachCorridorBegin)){
        vy = vySign * std::min(0.17 * agent->v, 1.0);
    }else{
        vy = -std::min(0.17 * agent->v, 1.0) * vySign;
    }
    return Action{ax, vy};
}

inline Action customizedMergingBehavior(const Environment& env, AgentPointer& agent){
    return customizedMergingBehavior(env, agent, agent->direction, agent->targetGapId);
}


//TODO: add probabilistic merging behavior
inline Action mergingBehaviorClosestGap(const Environment& env, AgentPointer& agent,
                                        const std::string& direction="left"){
    double ax = 0.;
    double vy = 0.;
    if(agent->corridorIdHistory.empty()){
        return Action{0., 0.};
    }
    const auto corridor = env.map.corridors.find(agent->corridorIdHistory.back())->second;
    // if distance to ego front < rss, do harsh brake -> only to make sure that the safety dis to front is kept
    AgentPointer frontAgent = env.frontAgent(agent);
    if(frontAgent){
        const double thw = (frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l)) / agent->v;
        agent->thwHistory.push_back(thw / agent->idmP.thw);
    }else{
        agent->thwHistory.push_back(1.0);
    }
    double desiredV = agent->desiredV;
    if(agent->isEgo){
        desiredV = std::min(1.1 * corridor.vLimit, agent->desiredV);
    }
    ax = idmAccFrontBack(agent->idmP, {frontAgent}, nullptr, agent->x, agent->v, agent->l, desiredV);
    double vySign = 1.;
    std::optional<int> targetCorridorId;
    if(direction == "left"){
        targetCorridorId = corridor.lId;
        if(!targetCorridorId){ std::cout << "CustomizedMergingBehavior: No corridor at left side!" << std::endl; }
    }else{
        targetCorridorId = corridor.rId;
        if(!targetCorridorId){ std::cout << "CustomizedMergingBehavior: No corridor at right side!" << std::endl; }
        vySign = -1.;
    }
//    std::cout << "target id: " << targetCorridorId.value() << std::endl;
    // reach beginning of target lane
    const auto targetCorridor = env.map.corridors.find(*targetCorridorId)->second;
    bool reachCorridorBegin = targetCorridor.reachCorridorBegin(agent->x);
    const double signedDisToCorridorBorder = direction == "left" ?
                                             corridor.signedDistanceToLeftBorder(agent->y, agent->d) :
                                             corridor.signedDistanceToRightBorder(agent->y, agent->d);
    bool farFromBorder = direction == "left" ? signedDisToCorridorBorder < -0.5: signedDisToCorridorBorder > 0.5;
    auto neighbor = direction == "left" ?
            env.neighborAgents(agent, true, false, false).left():
            env.neighborAgents(agent, false, true, false).right();
    if(neighbor.empty()){
        if(reachCorridorBegin || farFromBorder){
            return Action{ax, vySign * std::min(0.17 * agent->v, 1.0)};
        }
        return Action{ax, 0.};
    }
    // -0-- v1 --1-- v2 --2-- v3 --3--...
    std::map<int, double> gapWithTime;
    // merging to the very back
//        std::cout << " policy for id: " << agent->id << std::endl;
    gapWithTime[0] = timeToReachGap(-1e10, 0, neighbor[0]->x - 0.5 * (neighbor[0]->l + agent->l),
                                    neighbor[0]->v, agent->x, agent->v, desiredV);
//        std::cout << "time for gap: " << -1 << " " << leftVehicles[0]->id << " = " << gapWithTime.find(0)->second << std::endl;
    // merging to the very front
    gapWithTime[neighbor.size()] = timeToReachGap(neighbor.back()->x + 0.5 * (neighbor.back()->l + agent->l),
                                                  neighbor.back()->v, 1e10, 100, agent->x, agent->v, desiredV);
//        std::cout << "time for gap: " << leftVehicles.back()->id << " " << -1 << " = " << gapWithTime.find(leftVehicles.size())->second << std::endl;
    for(int i = 0; i < neighbor.size() - 1; ++i){
        const auto b = neighbor[i];
        const auto f = neighbor[i + 1];
        gapWithTime[i + 1] = timeToReachGap(b->x + 0.5 * (b->l + agent->l), b->v,
                                            f->x - 0.5 * (f->l + agent->l), f->v,
                                            agent->x, agent->v, desiredV);
//            std::cout << "time for gap: " << b->id << " " << f->id << " = " << gapWithTime.find(i + 1)->second << std::endl;
    }
    const int minId = std::min_element(gapWithTime.begin(), gapWithTime.end(),
                                       [](const auto& m, const auto& n) { return m.second < n.second; })->first;
    std::optional<double> ffd = std::nullopt;
    std::optional<double> ffv = std::nullopt;
    std::optional<double> fd = std::nullopt;
    std::optional<double> fv = std::nullopt;
    std::optional<double> bd = std::nullopt;
    std::optional<double> bv = std::nullopt;
    if(minId == 0){
        ax = idmAccFrontBack(agent->idmP, {neighbor[0]}, nullptr, agent->x, agent->v, agent->l, desiredV);
        if(neighbor.size() >= 2){
            ffd = neighbor[1]->x - agent->x - 0.5 * (neighbor[1]->l + agent->l);
            ffv = neighbor[1]->v;
        }
        fd = neighbor[0]->x - agent->x - 0.5 * (neighbor[0]->l + agent->l);
        fv = neighbor[0]->v;
    }else if (minId == neighbor.size()){
        ax = idmAccFrontBack(agent->idmP, {}, neighbor.back(), agent->x, agent->v, agent->l, desiredV);
        bd = neighbor.back()->x - agent->x + 0.5 * (neighbor.back()->l + agent->l);
        bv = neighbor.back()->v;
    }else{
        ax = idmAccFrontBack(agent->idmP, {neighbor[minId]}, neighbor[minId - 1], agent->x, agent->v,
                             agent->l, desiredV);
        if(neighbor.size() > minId + 1){
            ffd = neighbor[minId + 1]->x - agent->x - 0.5 * (neighbor[minId + 1]->l + agent->l);
            ffv = neighbor[minId + 1]->v;
        }
        fd = neighbor[minId]->x - agent->x - 0.5 * (neighbor[minId]->l + agent->l);
        fv = neighbor[minId]->v;
        bd = neighbor[minId - 1]->x - agent->x + 0.5 * (neighbor[minId - 1]->l + agent->l);
        bv = neighbor[minId - 1]->v;
    }
    bool rssSafe = rssSafeRelax(ffd, ffv, fd, fv, bd, bv, agent->v, desiredV, agent->rssP, agent->idmP);
    bool rssSafeFront = true;
    bool fallBackExists = corridor.fallBackExist(agent->x, agent->v, agent->l, agent->rssP.aMinBrake, agent->rssP.rTimeEgo);
    if(frontAgent && frontAgent->x - agent->x - 0.5 * (frontAgent->l + agent->l) <
                     rssDis(frontAgent->v, agent->v, agent->rssP.rTimeEgo, agent->rssP.aMinBrake, agent->rssP.aMaxBrake)){
        rssSafeFront = false;
    }
    if(!rssSafeFront || (!fallBackExists && !rssSafe)){
        ax = agent->rssP.aMinBrake;
    }
    if(farFromBorder || (rssSafe && rssSafeFront && reachCorridorBegin)){
        vy = vySign * std::min(0.17 * agent->v, 1.0);
    }else{
        vy = -std::min(0.17 * agent->v, 1.0) * vySign;
    }
    return Action{ax, vy};
}

inline Action mergingBehaviorClosestGap(const Environment& env, AgentPointer& agent){
    return mergingBehaviorClosestGap(env, agent, "left");
}


}

