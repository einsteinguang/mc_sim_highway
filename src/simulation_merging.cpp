#include "simulation_merging.hpp"

namespace mc_sim_highway{

bool SimulationMerging::actionWithId(const int& id, Action& action, bool printDebug) {
    IdmMatches front;
    IdmMatches egoFront;
    for(const auto& obj: neighbor_){
        if(std::find(frontVehicles_.begin(), frontVehicles_.end(), obj.first) != frontVehicles_.end()){
            front.push_back({obj.first, obj.second.x - ego_.x - 0.5 * (obj.second.l + ego_.l), obj.second.v});
        }
    }
    for(const auto& obj: egoFront_){
        egoFront.push_back({obj.id, obj.x - ego_.x - 0.5 * (obj.l + ego_.l), obj.v});
    }
    double axOnlyEgoFront = 100;
    if(!egoFront_.empty() && egoFront[0].dis <= rssDis(egoFront[0].v, ego_.v, p_.rTimeEgo, p_.aMinBrake, p_.aMaxBrake)){
        IdmParam emergencyIdm = ego_.idmP;
        emergencyIdm.accMin = p_.aMinBrake;
        axOnlyEgoFront = idmAccFrontBack(emergencyIdm, egoFront, {-1, 0., 0.}, ego_.v, map_.vd);
    }
    double ax = -100;
    double vy = -100;
    if(id == -1){
        if(!initialActions_.empty()){
            const auto obj = neighbor_.find(initialActions_.back())->second;
            ax = idmAccFrontBack(ego_.idmP, {{obj.id, obj.x - ego_.x - 0.5 * (obj.l + ego_.l), obj.v}},
                                 {-1, 0., 0.}, ego_.v, map_.vd);
            vy = lateralVelocity({{obj.id, obj.x - ego_.x - 0.5 * (obj.l + ego_.l), obj.v}}, {-1, 0., 0.}, ego_);
        } else{
            ax = idmAccFrontBack(ego_.idmP, front, {-1, 0., 0.}, ego_.v, map_.vd);
            vy = lateralVelocity(front, {-1, 0., 0.}, ego_);
        }
    }else{
        for(const auto& actionId: initialActions_){
            if(id == actionId){
                const auto obj = neighbor_.find(actionId)->second;
                const IdmMatch back = {obj.id, obj.x - ego_.x + 0.5 * (obj.l + ego_.l), obj.v};
                auto newFront = front;
                if(actionId >= 1){
                    const auto f = neighbor_.find(actionId - 1)->second;
                    newFront.insert(newFront.begin(), {f.id, f.x - ego_.x - 0.5 * (f.l + ego_.l), f.v});
                }
                ax = idmAccFrontBack(ego_.idmP, newFront, back, ego_.v, map_.vd);
                vy = lateralVelocity(newFront, back, ego_);
                break;
            }
        }
    }
    if(printDebug){
        std::cout << "action id: " << id << " rss safe: " << rssSafeForLaneChange_[ego_.id] << std::endl;
    }
    if (ax != -100 || vy != -100){
        if((!ego_.fallBackExist(map_, p_.rTimeEgo)) && !rssSafeForLaneChange_[ego_.id]){
            axOnlyEgoFront = std::min(axOnlyEgoFront, ego_.idmP.aEmergency);
        }
        action = {std::min(ax, axOnlyEgoFront), vy};
        return true;
    }
    std::cout << "No action index " << id << std::endl;
    return false;
}


bool SimulationMerging::actionWithIdEgoFront(const int &id, Action &action, Vehicle& egoFront, bool printDebug) {
    IdmMatches front;
    for(const auto& obj: neighborEgoFront_){
        if(std::find(frontVehiclesEgoFront_.begin(), frontVehiclesEgoFront_.end(), obj.first) != frontVehiclesEgoFront_.end()){
            front.push_back({obj.first, obj.second.x - ego_.x - 0.5 * (obj.second.l + ego_.l), obj.second.v});
        }
    }
    double ax = -100;
    double vy = -100;
    if(id == -1){
        if(!initialActionsEgoFront_.empty()){
            const auto obj = neighborEgoFront_.find(initialActionsEgoFront_.back())->second;
            ax = idmAccFrontBack(egoFront.idmP, {{obj.id, obj.x - egoFront.x - 0.5 * (obj.l + egoFront.l), obj.v}},
                                 {-1, 0., 0.}, egoFront.v, map_.vd);
            vy = lateralVelocity({{obj.id, obj.x - egoFront.x - 0.5 * (obj.l + egoFront.l), obj.v}}, {-1, 0., 0.}, egoFront);
        } else{
            ax = idmAccFrontBack(egoFront.idmP, front, {-1, 0., 0.}, egoFront.v, map_.vd);
            vy = lateralVelocity(front, {-1, 0., 0.}, egoFront);
        }
    }else{
        for(const auto& actionId: initialActionsEgoFront_){
            if(id == actionId){
                const auto obj = neighborEgoFront_.find(actionId)->second;
                const IdmMatch back = {obj.id, obj.x - egoFront.x + 0.5 * (obj.l + egoFront.l), obj.v};
                auto newFront = front;
                if(actionId >= 1){
                    const auto f = neighborEgoFront_.find(actionId - 1)->second;
                    newFront.insert(newFront.begin(), {f.id, f.x - egoFront.x - 0.5 * (f.l + egoFront.l), f.v});
                }
                ax = idmAccFrontBack(egoFront.idmP, newFront, back, egoFront.v, map_.vd);
                vy = lateralVelocity(newFront, back, egoFront);
                break;
            }
        }
    }
    if(printDebug){
        std::cout << "action id: " << id << " rss safe: " << rssSafeForLaneChange_[egoFront.id] << std::endl;
    }
    if (ax != -100 || vy != -100){
        if((!egoFront.fallBackExist(map_, p_.rTimeOther)) && !rssSafeForLaneChange_[egoFront.id]){
            ax = std::min(ax, egoFront.idmP.aEmergency);
        }
        action = {ax, vy};
        return true;
    }
    std::cout << "No action index " << id << std::endl;
    return false;
}


void SimulationMerging::step(const Action& action) {
    Vehicles vehicles = {ego_};
    ego_.step(action.ax, action.vy, dt_);
    for(auto& obj: egoFront_){
        if (egoFrontSimpleMergingPolicy_){
            const auto actionIdEgoFront = closestGapEgoFront(obj);
            Action egoFrontAction{};
            actionWithIdEgoFront(actionIdEgoFront, egoFrontAction, obj);
            obj.step(egoFrontAction.ax, egoFrontAction.vy, dt_);
        } else{
            obj.step(0, 0, dt_);
        }
        vehicles.push_back(obj);
    }
    for(auto& obj: neighbor_){
        std::vector<double> probabilities;
        for(const auto& vehicle : vehicles){
            const double d = vehicle.x - obj.second.x - 0.5 * (obj.second.l + vehicle.l);
            const double thw = d / std::max(obj.second.v, 1e-10);
            const double ttc = noUnitDistanceChangingRate(obj.second.v, vehicle.v);
            probabilities.push_back(yieldingProbabilityLearned({thw, ttc, obj.second.a}));
//            std::cout << obj.second.id << " yield to " << vehicle.id << " p: " << yieldingProbabilityLearned({thw, ttc, obj.second.initialA}) << std::endl;
//            std::cout << d << " " << thw << " " << ttc << " " << obj.second.a << std::endl;
        }
        for(int i = 0; i < probabilities.size(); i++){
            if(obj.second.yieldingIntentionToOthers.find(vehicles[i].id) == obj.second.yieldingIntentionToOthers.end()){
                obj.second.yieldingIntentionToOthers.insert({vehicles[i].id, YieldingIntention(vehicles[i].id, probabilities[i])});
            } else{
                obj.second.yieldingIntentionToOthers.find(vehicles[i].id)->second.updateIntention(probabilities[i]);
                if (vehicles[i].finishedMerging(map_) && obj.second.x < vehicles[i].x){
                    obj.second.yieldingIntentionToOthers.find(vehicles[i].id)->second.yielding = true;
                }
            }
        }
        IdmMatches frontObjs;
        if(obj.first > 0){
            const auto o = neighbor_.find(obj.first - 1)->second;
            frontObjs.push_back({o.id, o.x - obj.second.x - 0.5 * (o.l + obj.second.l), o.v});
        }
        for(const auto& vehicle: vehicles){
//            std::cout << obj.second.id << " yield to " << vehicle.id << " " << obj.second.yieldingIntentionToOthers.find(vehicle.id)->second.yielding << std::endl;
            if(obj.second.yieldingIntentionToOthers.find(vehicle.id)->second.yielding){
                frontObjs.push_back({vehicle.id, vehicle.x - obj.second.x - 0.5 * (vehicle.l + obj.second.l), vehicle.v});
            }
        }
        if(step_ == 0){
            obj.second.step(obj.second.a, 0., dt_);
            if(neighborEgoFront_.find(obj.first) != neighborEgoFront_.end()){
                neighborEgoFront_.find(obj.first)->second.step(obj.second.a, 0., dt_);
            }
        }else{
            const double acc = idmAccFrontBack(obj.second.idmP, frontObjs, {-1, 0., 0.}, obj.second.v, map_.vd);
//            std::cout << "front obj size: " << frontObjs.size() << " " << obj.second.id << " acc " << acc << std::endl;
            obj.second.step(acc, 0, dt_);
            if(neighborEgoFront_.find(obj.first) != neighborEgoFront_.end()){
                neighborEgoFront_.find(obj.first)->second.step(acc, 0., dt_);
            }
        }
    }
}

SimulationMerging::TerminationStatus SimulationMerging::runOneSimulation(const int& initialEgoAction) {
    // utility: average speed to vd, penalize for v > vd as well
    // comfort: average deceleration(acceleration) / d_max(a_max)
    // initialEgoAction: vehicle id before which we want to merge in
    TerminationStatus status = checkTermination({0, 0.5});  // assume the initial state is safe
    history_.egoPositionHistory.emplace_back(ego_.x, ego_.y);
    for(const auto& obj: neighbor_){
        history_.neighborPositionHistory[obj.first] = {{obj.second.x, obj.second.y}};
    }
    for(const auto& obj: egoFront_){
        history_.neighborPositionHistory[obj.id] = {{obj.x, obj.y}};
    }
    while (!terminated(status)){
        Action action{};
        const bool actionExists = actionWithId(initialEgoAction, action);
        if (!actionExists){
            status.error = true;
            return status;
        }
        step(action);
        history_.egoPositionHistory.emplace_back(ego_.x, ego_.y);
        for(const auto& obj: neighbor_){
            history_.neighborPositionHistory[obj.first].push_back({obj.second.x, obj.second.y});
        }
        for(const auto& obj: egoFront_){
            history_.neighborPositionHistory[obj.id].push_back({obj.x, obj.y});
        }
        utility_ = utility_ + stepUtility();
        comfort_ = comfort_ + stepComfort(action.ax);
        status = checkTermination(action);
        step_++;
    }
    utility_ = utility_ / double(std::max(step_, 1));
    comfort_ = comfort_ / double(std::max(step_, 1));
    return status;
}

SimulationMerging::TerminationStatus SimulationMerging::runOneSimulationWithoutHistory(const int &initialEgoAction) {
    // utility: average speed to vd, penalize for v > vd as well
    // comfort: average deceleration(acceleration) / d_max(a_max)
    // initialEgoAction: vehicle id before which we want to merge in
    TerminationStatus status = checkTermination({0, 0});
    while (!terminated(status)){
        Action action{};
        const bool actionExists = actionWithId(initialEgoAction, action);
        if (!actionExists){
            status.error = true;
            return status;
        }
        step(action);
        utility_ = utility_ + stepUtility();
        comfort_ = comfort_ + stepComfort(action.ax);
        status = checkTermination(action);
        step_++;
    }
    utility_ = utility_ / double(std::max(step_, 1));
    comfort_ = comfort_ / double(std::max(step_, 1));
    return status;
}

}  // namespace mc_sim_highway
