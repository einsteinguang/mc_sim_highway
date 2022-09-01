#ifndef SRC_VEHICLE_HPP
#define SRC_VEHICLE_HPP
#pragma once
#include "utility.hpp"

namespace mc_sim_highway{

struct Vehicle {
    Vehicle(const int& id, const double& x, const double& y, const double& v, const double& a,
            const double& l, const double& d, const IdmParam& p):
            id(id), x(x), y(y), v(v), a(a), l(l), d(d), idmP(p){
        accHistory.push_back(a);
    }

    int id;
    double x;
    double y;
    double v;
    double a;
    double l;
    double d;
    IdmParam idmP;

    std::vector<double> accHistory;

    std::map<int, YieldingIntention> yieldingIntentionToOthers;

    void step(const double& ax, const double& vy, const double& dt){
        v = std::max(v + ax * dt, 0.);
        x = x + v * dt;
        y = y + vy * dt;
        a = ax;
        accHistory.push_back(ax);
    }

    void stepIdm(const IdmMatches& fs, const IdmMatch& b, const double& vd, const double& vy, const double& dt){
        const double acc = idmAccFrontBack(idmP, fs, b, v, vd);
        a = acc;
//        if(id == 1){
//            std::cout << "acc: " << acc << std::endl;
//        }
        step(acc, vy, dt);
        accHistory.push_back(acc);
    }

    bool collideWith(const Vehicle& car) const{
        const double lon = 0.5 * (l + car.l);
        const double lat = 0.5 * (d + car.d);
        return !(x - car.x > lon || x - car.x < -lon || y - car.y > lat || y - car.y < -lat);
    }

    bool finishedMerging(const MapMerging& map) const{
        return y >= 0.5 * map.targetLane.p1.y;
    }

    double timeToFinishMerging(const MapMerging& map, const double& vy) const{
        if(vy <= 0){
            return 1e10;
        }
        return (0.8 * map.targetLane.p1.y - y) / vy;
    }

    double timeToReachMergingLaneEnd(const MapMerging& map) const{
        if(v == 0){
            return 1e10;
        }
        return (map.mergingLane.p2.x - x) / v;
    }

    bool fallBackExist(const MapMerging& map, const double& reactionT) const{
        const double disToStop = v * reactionT + v * v / (-2 * idmP.aEmergency);
        return disToStop <= map.mergingLane.p2.x - x - 0.8 * l;
    }
};

using Vehicles = std::vector<Vehicle>;

} // namespace mc_sim_highway

#endif // SRC_VEHICLE_HPP
