#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include "simulation_merging.hpp"
#include "simulation_multilane.hpp"

using namespace boost::python;
using namespace mc_sim_highway;

namespace python_converter{

template <typename T>
struct VectorToList {
    static PyObject* convert(const T& v) {
        list l;
        for (auto& e : v) {
            l.append(e);
        }
        return incref(l.ptr());
    }
};
template <typename T>
using VectorToListConverter = to_python_converter<T, VectorToList<T>>;

template< typename T >
inline std::vector<T> vectorFromList( const boost::python::list& iterable ){
    return std::vector<T>( boost::python::stl_input_iterator<T>(iterable ),
            boost::python::stl_input_iterator<T>() );
}

Indicator simulationMergingWrapper(const boost::python::list& objectList,
                                   const boost::python::list& egoF,
                                   const Vehicle& ego,
                                   const MapMerging& map,
                                   const int& actionId,
                                   const RSSParam& p,
                                   const int& maxStep){
    Vehicles vehicles{boost::python::stl_input_iterator<Vehicle>(objectList),
                      boost::python::stl_input_iterator<Vehicle>()};
    Vehicles egoFront{boost::python::stl_input_iterator<Vehicle>(egoF),
                      boost::python::stl_input_iterator<Vehicle>()};
    return simulationMerging(vehicles, egoFront, ego, map, actionId, p, maxStep);
}

Indicator simulationMergingMultiThreadWrapper(const boost::python::list& objectList,
                                              const boost::python::list& egoF,
                                              const Vehicle& ego,
                                              const MapMerging& map,
                                              const int& actionId,
                                              const RSSParam& p,
                                              const int& maxStep,
                                              bool egoFrontMergingPolicy=true){
    Vehicles vehicles{boost::python::stl_input_iterator<Vehicle>(objectList),
                      boost::python::stl_input_iterator<Vehicle>()};
    Vehicles egoFront{boost::python::stl_input_iterator<Vehicle>(egoF),
                      boost::python::stl_input_iterator<Vehicle>()};
    return simulationMergingMultiThread(vehicles, egoFront, ego, map, actionId, p, maxStep, egoFrontMergingPolicy);
}

BehaviorFeature simulationMultiThreadWrapper(const int& nEachThread,
                                             const SimParameter& param,
                                             const MapMultiLane& map,
                                             const boost::python::list& as,
                                             const int& egoId,
                                             const std::string& targetDirection,
                                             const int& targetGapId){
    Agents vehicles{boost::python::stl_input_iterator<Agent>(as), boost::python::stl_input_iterator<Agent>()};
    return runMultiThreads(nEachThread, param, map, vehicles, egoId, targetDirection, targetGapId);
}

Action getMergingActionWrapper(const boost::python::list& objectList,
                               const boost::python::list& egoF,
                               const Vehicle& ego,
                               const MapMerging& map,
                               const int& actionId,
                               const RSSParam& p){
    Vehicles vehicles{boost::python::stl_input_iterator<Vehicle>(objectList),
                      boost::python::stl_input_iterator<Vehicle>()};
    Vehicles egoFront{boost::python::stl_input_iterator<Vehicle>(egoF),
                      boost::python::stl_input_iterator<Vehicle>()};
    return getMergingAction(vehicles, egoFront, ego, map, actionId, p);
}

Action getMergingActionClosestGapWrapper(const boost::python::list& objectList,
                                         const boost::python::list& egoF,
                                         const Vehicle& ego,
                                         const MapMerging& map,
                                         const RSSParam& p){
    Vehicles vehicles{boost::python::stl_input_iterator<Vehicle>(objectList),
                      boost::python::stl_input_iterator<Vehicle>()};
    Vehicles egoFront{boost::python::stl_input_iterator<Vehicle>(egoF),
                      boost::python::stl_input_iterator<Vehicle>()};
    return getMergingActionClosestGap(vehicles, egoFront, ego, map, p);
}

int getMergingActionClosestGapIdWrapper(const boost::python::list& objectList,
                                        const boost::python::list& egoF,
                                        const Vehicle& ego,
                                        const MapMerging& map,
                                        const RSSParam& p){
    Vehicles vehicles{boost::python::stl_input_iterator<Vehicle>(objectList),
                      boost::python::stl_input_iterator<Vehicle>()};
    Vehicles egoFront{boost::python::stl_input_iterator<Vehicle>(egoF),
                      boost::python::stl_input_iterator<Vehicle>()};
    return getMergingActionClosestGapId(vehicles, egoFront, ego, map, p);
}

PositionHistory simulationMergingOnceWrapper(const boost::python::list& objectList,
                                             const boost::python::list& egoF,
                                             const Vehicle& ego,
                                             const MapMerging& map,
                                             const int& actionId,
                                             const RSSParam& p,
                                             bool printDebug = false,
                                             int printId = -1){
    Vehicles vehicles{boost::python::stl_input_iterator<Vehicle>(objectList),
                      boost::python::stl_input_iterator<Vehicle>()};
    Vehicles egoFront{boost::python::stl_input_iterator<Vehicle>(egoF),
                      boost::python::stl_input_iterator<Vehicle>()};
    return simulationMergingOnce(vehicles, egoFront, ego, map, actionId, p, printDebug, printId);
}

bool rssSafeWrapper(const boost::python::list& fs, const IdmMatch& back,
                    const double& v0, const double vd, const double& backVehicleSoftDcc,
                    const RSSParam& rssP, const IdmParam& idmP){
    IdmMatches fronts{boost::python::stl_input_iterator<IdmMatch>(fs),
                      boost::python::stl_input_iterator<IdmMatch>()};
    return rssSafe(fronts, back, v0, vd, backVehicleSoftDcc, rssP, idmP);
}

bool rssSafeRelaxWrapper(const boost::python::list& fs, const IdmMatch& back,
                    const double& v0, const double vd,
                    const RSSParam& rssP, const IdmParam& idmP){
    IdmMatches fronts{boost::python::stl_input_iterator<IdmMatch>(fs),
                      boost::python::stl_input_iterator<IdmMatch>()};
    return rssSafeRelax(fronts, back, v0, vd, rssP, idmP);
}

static list keys(const ObjHistoryMap& map){
    list t;
    for(const auto & it : map){
        t.append(it.first);
    }
    return t;
}

ActionWithType laneChangeBehaviorMOBILWrapper(const Environment& env, const int& agentId,
                                              const Action& actionIdm,
                                              bool safetyRequired){
    const auto agent = env.agents.find(agentId);
    if(agent == env.agents.end()){
        std::cout << "No agent with id: " << agentId << std::endl;
    }
    ActionWithType decision;
    AgentPointer agentPtr = agent->second;
    auto d = laneChangeBehaviorMOBIL(env, agentPtr, actionIdm,
                                     decision.commitToDecision,
                                     decision.commitKeepLaneStep,
                                     decision.targetLaneId,
                                     false,
                                     safetyRequired);
    decision.ax = d.ax;
    decision.vy = d.vy;
    return decision;
}

Action customizedLaneChangeBehaviorWrapper(const Environment& env, const int& agentId, const std::string& behavior){
    const auto agent = env.agents.find(agentId);
    if(agent == env.agents.end()){
        std::cout << "No agent with id: " << agentId << std::endl;
    }
    AgentPointer agentPtr = agent->second;
    return customizedLaneChangeBehavior(env, agentPtr, behavior);
}

Action customizedMergingBehaviorWrapper(const Environment& env, const int& agentId,
                                        const std::string& behavior, const int& targetGapId){
    const auto agent = env.agents.find(agentId);
    if(agent == env.agents.end()){
        std::cout << "No agent with id: " << agentId << std::endl;
    }
    AgentPointer agentPtr = agent->second;
    return customizedMergingBehavior(env, agentPtr, behavior, targetGapId);
}

Action closestGapMergingBehaviorWrapper(const Environment& env, const int& agentId,
                                        const std::string& direction){
    const auto agent = env.agents.find(agentId);
    if(agent == env.agents.end()){
        std::cout << "No agent with id: " << agentId << std::endl;
    }
    AgentPointer agentPtr = agent->second;
    return mergingBehaviorClosestGap(env, agentPtr, direction);
}

}  // namespace python_converter


BOOST_PYTHON_MODULE(mc_sim_highway)
{
    class_<Point>("Point", init<double, double>())
            .def_readwrite("x", &Point::x)
            .def_readwrite("y", &Point::y);
    class_<Points>("Points")
            .def("__iter__", iterator<Points>());

    class_<State>("State", no_init)
            .def_readwrite("x", &State::x)
            .def_readwrite("y", &State::y)
            .def_readwrite("v", &State::v)
            .def_readwrite("a", &State::a);
    class_<States>("States")
            .def("__iter__", iterator<States>());

    class_<ObjHistoryMap>("ObjHistoryMap")
            .def(map_indexing_suite<ObjHistoryMap>());
    class_<ObjHistoryStatesMap>("ObjHistoryStatesMap")
            .def(map_indexing_suite<ObjHistoryStatesMap>());

    class_<PositionHistory>("PositionHistory", no_init)
        .def_readonly("success", &PositionHistory::success)
        .def_readonly("egoHistory", &PositionHistory::egoPositionHistory)
        .def_readonly("neighborHistory", &PositionHistory::neighborPositionHistory);

    class_<Line>("Line", init<Point, Point>())
        .def_readonly("p1", &Line::p1)
        .def_readonly("p2", &Line::p2);

    class_<Action>("Action", no_init)
            .def("__init__", make_constructor(
                    +[](double ax, double vy) {
                        return std::make_shared<Action>(Action({ax, vy}));
                    },
                    default_call_policies(), (arg("ax") = 0., arg("vy") = 0.)))
            .def_readonly("ax", &Action::ax)
            .def_readonly("vy", &Action::vy);

    class_<ActionWithType>("ActionWithType", no_init)
            .def_readonly("ax", &ActionWithType::ax)
            .def_readonly("vy", &ActionWithType::vy)
            .def_readonly("commitToDecision", &ActionWithType::commitToDecision)
            .def_readonly("commitKeepLaneStep", &ActionWithType::commitKeepLaneStep)
            .def_readonly("targetLaneId", &ActionWithType::targetLaneId);

    class_<MapMerging>("MapMerging", init<Line, Line, double>())
            .def_readwrite("vd", &MapMerging::vd)
            .def_readonly("mergingLane", &MapMerging::mergingLane)
            .def_readonly("targetLane", &MapMerging::targetLane);

    class_<Corridor>("Corridor", init<int, std::string, Line, Line, Line, double>())
            .def("setLeftAndRightCorridorId", &Corridor::setLeftAndRightCorridorIdWrapper)
            .def_readwrite("id", &Corridor::id)
            .def_readwrite("l", &Corridor::l)
            .def_readwrite("r", &Corridor::r)
            .def_readwrite("m", &Corridor::m)
            .def_readwrite("type", &Corridor::type)
            .def_readwrite("vLimit", &Corridor::vLimit);
    python_converter::VectorToListConverter<Corridors>();

    class_<MapMultiLane>("MapMultiLane", no_init)
            .def("__init__", make_constructor(
                    +[](const boost::python::list& corridors) {
                        return std::make_shared<MapMultiLane>(MapMultiLane{
                            python_converter::vectorFromList<Corridor>(corridors)});
                    },
                    default_call_policies(),
                    (arg("corridors") = boost::python::list())));

    class_<IdmParam>("IdmParam", init<double, double, double, double, double, double>(
            (arg("am") = 1.5, arg("amin") = -4., arg("minD") = 2., arg("aCom") = -2., arg("aE") = -7., arg("t") = 1.5)))
            .def_readwrite("accMax", &IdmParam::accMax)
            .def_readwrite("accMin", &IdmParam::accMin)
            .def_readwrite("minDis", &IdmParam::minDis)
            .def_readwrite("accCom", &IdmParam::accCom)
            .def_readwrite("aEmergency", &IdmParam::aEmergency)
            .def_readwrite("thw", &IdmParam::thw);

    class_<RSSParam>("RSSParam", init<double, double, double, double, double, double>(
            (arg("rTOther") = 1., arg("rTEgo") = 0.2, arg("aMin") = -7.0,
                    arg("aMax") = -5.0, arg("aSoft") = 1.8, arg("dSoft") = 1.2)))
            .def_readwrite("rTimeOther", &RSSParam::rTimeOther)
            .def_readwrite("rTimeEgo", &RSSParam::rTimeEgo)
            .def_readwrite("aMinBrake", &RSSParam::aMinBrake)
            .def_readwrite("aMaxBrake", &RSSParam::aMaxBrake)
            .def_readwrite("a", &RSSParam::a)
            .def_readwrite("d", &RSSParam::d);

    class_<YieldingParam, std::shared_ptr<YieldingParam>>("YieldingParam", no_init)
            .def("__init__", make_constructor(
                    +[](double bias, double a, double b, double c, double pf, double da, double ba) {
                        return std::make_shared<YieldingParam>(YieldingParam({bias, a, b, c, pf, da, ba}));
                    },
                    default_call_policies(),
                    (arg("bias") = -0.5, arg("a") = 1.81, arg("b") = -4.8, arg("c") = -1.1,
                            arg("pf") = 0.9, arg("da") = 0.5, arg("ba") = 0.51)))
            .def_readwrite("bias", &YieldingParam::bias)
            .def_readwrite("a", &YieldingParam::a)
            .def_readwrite("b", &YieldingParam::b)
            .def_readwrite("c", &YieldingParam::c)
            .def_readwrite("politenessFactor", &YieldingParam::politenessFactor)
            .def_readwrite("deltaA", &YieldingParam::deltaA)
            .def_readwrite("biasA", &YieldingParam::biasA);

    class_<SimParameter, std::shared_ptr<SimParameter>>("SimParameter", no_init)
            .def("__init__", make_constructor(
                    +[](double dt, int steps, int minSteps) {
                        return std::make_shared<SimParameter>(SimParameter({dt, steps, minSteps}));
                    },
                    default_call_policies(), (arg("dt") = 0.3, arg("steps") = 30, arg("minSteps") = 15)))
            .def_readwrite("dt", &SimParameter::dt)
            .def_readwrite("steps", &SimParameter::totalSteps)
            .def_readwrite("minSteps", &SimParameter::minSteps);

    class_<IdmMatch, std::shared_ptr<IdmMatch>>("IdmMatch", no_init)
            .def("__init__", make_constructor(
                    +[](int id, double dis, double v) {
                        return std::make_shared<IdmMatch>(IdmMatch({id, dis, v}));
                    },
                    default_call_policies(), (arg("id") = 0, arg("dis") = 0., arg("v") = 0.)))
            .def_readwrite("id", &IdmMatch::id)
            .def_readwrite("dis", &IdmMatch::dis)
            .def_readwrite("v", &IdmMatch::v);
    python_converter::VectorToListConverter<IdmMatches>();

    class_<Vehicle>("Vehicle", init<int, double, double, double, double, double, double, IdmParam>())
            .def_readonly("id", &Vehicle::id)
            .def_readonly("x", &Vehicle::x)
            .def_readonly("y", &Vehicle::y)
            .def_readonly("v", &Vehicle::v)
            .def_readonly("a", &Vehicle::a)
            .def_readonly("l", &Vehicle::l)
            .def_readonly("d", &Vehicle::d)
            .def_readonly("idmP", &Vehicle::idmP);
    python_converter::VectorToListConverter<Vehicles>();

    class_<Agent>("Agent", init<int, double, double, double, double, double, double, double, double,
            IdmParam, RSSParam, YieldingParam, std::string>())
            .def_readwrite("targetLaneId", &Agent::targetLaneId)
            .def_readwrite("commitToDecision", &Agent::commitToDecision)
            .def_readwrite("commitKeepLaneStep", &Agent::commitKeepLaneStep)
            .def_readonly("id", &Agent::id)
            .def_readonly("statesHistory", &Agent::statesHistory)
            .def("setDesiredV", &Agent::setDesiredV);
    python_converter::VectorToListConverter<Agents>();

    class_<Environment>("Environment", no_init)
            .def("__init__", make_constructor(
                    +[](const MapMultiLane& map, const boost::python::list& agents) {
                        return std::make_shared<Environment>(Environment{
                            map, python_converter::vectorFromList<Agent>(agents)});
                    },
                    default_call_policies(),
                    (arg("map") = MapMultiLane(),
                     arg("agents") = boost::python::list())))
            .def("reset", &Environment::reset)
            .def("setEgoAgentIdAndDrivingBehavior", &Environment::setEgoAgentIdAndDrivingBehavior,
                 (boost::python::arg("targetGapId")=(int)(-2)));

    class_<Simulation>("Simulation", no_init)
            .def("__init__", make_constructor(
                    +[](const SimParameter& simParam, const MapMultiLane& map, const boost::python::list& agents) {
                        return std::make_shared<Simulation>(Simulation{
                            simParam, map, python_converter::vectorFromList<Agent>(agents)});
                    },
                    default_call_policies(),
                    (arg("simParam") = SimParameter(),
                            arg("map") = MapMultiLane(),
                            arg("agents") = boost::python::list())))
            .def("agentsStatesHistory", &Simulation::agentsStatesHistory)
            .def("run", &Simulation::run)
            .def("runMultiTimes", &Simulation::runMultipleTimes)
            .def("reset", &Simulation::reset)
            .def("setEgoAgentIdAndDrivingBehavior", &Simulation::setEgoAgentIdAndDrivingBehavior,
                 (boost::python::arg("targetGapId")=(int)(-2)));

    class_<Indicator, std::shared_ptr<Indicator>>("Indicator", no_init)
            .def_readwrite("successRate", &Indicator::successRate)
            .def_readwrite("fallBackRate", &Indicator::fallBackRate)
            .def_readwrite("utility", &Indicator::utility)
            .def_readwrite("comfort", &Indicator::comfort)
            .def_readwrite("successSpeedRatio", &Indicator::expectedStepRatio)
            .def_readwrite("normalizedAccObjects", &Indicator::normalizedAccObjects)
            .def_readonly("error", &Indicator::error);

    class_<BehaviorFeature, std::shared_ptr<BehaviorFeature>>("BehaviorFeature", no_init)
            .def_readwrite("successRate", &BehaviorFeature::successRate)
            .def_readwrite("fallBackRate", &BehaviorFeature::fallBackRate)
            .def_readwrite("utility", &BehaviorFeature::utility)
            .def_readwrite("comfort", &BehaviorFeature::comfort)
            .def_readwrite("expectedStepRatio", &BehaviorFeature::expectedStepRatio)
            .def_readwrite("utilityObjects", &BehaviorFeature::utilityObjects)
            .def_readwrite("comfortObjects", &BehaviorFeature::comfortObjects)
            .def_readwrite("fromNSims", &BehaviorFeature::fromNSims);

    class_<StatusOneSim, std::shared_ptr<StatusOneSim>>("StatusOneSim", no_init)
            .def_readwrite("finish", &StatusOneSim::finish)
            .def_readwrite("fallBack", &StatusOneSim::fallBack)
            .def_readwrite("utility", &StatusOneSim::utility)
            .def_readwrite("comfort", &StatusOneSim::comfort)
            .def_readwrite("finishStep", &StatusOneSim::finishStep)
            .def_readwrite("utilityObjects", &StatusOneSim::utilityObjects)
            .def_readwrite("comfortObjects", &StatusOneSim::comfortObjects);

    def("simulationMerging", python_converter::simulationMergingWrapper);
    def("simulationMergingMultiThread", python_converter::simulationMergingMultiThreadWrapper);
    def("simulationMergingOnce", python_converter::simulationMergingOnceWrapper);
    def("getMergingAction", python_converter::getMergingActionWrapper);
    def("getMergingActionClosestGap", python_converter::getMergingActionClosestGapWrapper);
    def("getMergingActionClosestGapId", python_converter::getMergingActionClosestGapIdWrapper);
    def("rssSafe", python_converter::rssSafeWrapper);
    def("rssSafeRelax", python_converter::rssSafeRelaxWrapper);
    def("simulationMultiThread", python_converter::simulationMultiThreadWrapper);

    // new
    def("laneChangeMobilBehavior", python_converter::laneChangeBehaviorMOBILWrapper);
    def("fixedDirectionLaneChangeBehavior", python_converter::customizedLaneChangeBehaviorWrapper);
    def("fixedGapMergingBehavior", python_converter::customizedMergingBehaviorWrapper);
    def("closestGapMergingBehavior", python_converter::closestGapMergingBehaviorWrapper);
}

