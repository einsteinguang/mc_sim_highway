// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include "gtest/gtest.h"

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(${pkgname}, TestName) {
//     EXPECT_TRUE(true);
//     TODO: Add your test code here
//}
#include <chrono>
#include "simulation_merging.hpp"
#include "simulation_multilane.hpp"


TEST(MCS, SimulationMultilane){
    using namespace mc_sim_highway;
    Corridor merging(0, "merging", {{0, 1.5}, {200, 1.5}}, {{0, -1.5}, {200, -1.5}}, {{0, 0}, {200, 0}}, 20);
    Corridor main(1, "main", {{0, 4.5}, {1000, 4.5}}, {{0, 1.5}, {1000, 1.5}}, {{0, 3}, {1000, 3}}, 20);
    Corridor main2(2, "main", {{0, 7.5}, {1000, 7.5}}, {{0, 4.5}, {1000, 4.5}}, {{0, 6}, {1000, 6}}, 20);
    MapMultiLane map{{merging, main, main2}};

    const IdmParam idmP{2., -4., 2., -2., -7., 1.5};
    const RSSParam rssP{0.8, 0.4, -8., -10., 1.8, 1.2};
    const YieldingParam yp;
    Agent a0{0, 15, -0.5, 20, 0, 20, 0, 5, 2, idmP, rssP, yp, "merging"};
    Agent a1{1, 55, -0.2, 20, 0, 20, 0, 5, 2, idmP, rssP, yp, "merging"};
    Agent a2{2, 5, 3.1, 20, 0, 20, 0, 5, 2, idmP, rssP, yp, "idm_lc"};
    Agent a3{3, 45, 3.2, 20, 0, 20, 0, 5, 2, idmP, rssP, yp, "idm_lc"};
    Agent a4{4, 80, 6.1, 20, 0, 20, 0, 5, 2, idmP, rssP, yp, "idm_lc"};
    Agent a5{5, 120, 6.2, 20, 0, 20, 0, 5, 2, idmP, rssP, yp, "idm_lc"};

    SimParameter simParam;
    Simulation sim(simParam, map, {a0, a1, a2, a3, a4, a5});

    auto tStart = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 100; i++){
        sim.run();
        sim.reset();
    }
    auto tEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Simulation time (ms): " << std::chrono::duration<double, std::milli>(tEnd - tStart).count() << std::endl;

    tStart = std::chrono::high_resolution_clock::now();
    sim.setEgoAgentIdAndDrivingBehavior(2, "left");
    auto feature = sim.runMultipleTimes(180);
    std::cout << "feature: " << feature.successRate << " " << feature.fallBackRate << " " << feature.comfort << " " <<
              feature.utility << " " << feature.expectedStepRatio << " " << feature.utilityObjects << " " << feature.comfortObjects << std::endl;
    tEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Simulation time (ms): " << std::chrono::duration<double, std::milli>(tEnd - tStart).count() << std::endl;

    tStart = std::chrono::high_resolution_clock::now();
    feature = runMultiThreads(50, simParam, map, {a0, a1, a2, a3, a4, a5}, 2, "left");
    std::cout << "feature: " << feature.successRate << " " << feature.fallBackRate << " " << feature.comfort << " " <<
              feature.utility << " " << feature.expectedStepRatio << " " << feature.utilityObjects << " " << feature.comfortObjects << std::endl;
    tEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Simulation time (ms): " << std::chrono::duration<double, std::milli>(tEnd - tStart).count() << std::endl;
}

TEST(MCS, timeToReachGap){
    using namespace mc_sim_highway;
    std::cout << timeToReachGap(-10., 7, 5., 7., 0., 10.) << std::endl;
    std::cout << timeToReachGap(-1000., 0, -5, 7., 0., 10.) << std::endl;
    std::cout << timeToReachGap(10, 7, 1000, 100., 0., 10.) << std::endl;

    const auto p1 = moveBasedLaneChangeProbability(0., 0.);
    const auto p2 = moveBasedLaneChangeProbability(0.3, 0.3);
    const auto p3 = moveBasedLaneChangeProbability(0., 0.3);
    const auto p4 = moveBasedLaneChangeProbability(0.3, 0.);
    std::cout << p1.left << " " << p1.keep << " " << p1.right << std::endl;
    std::cout << p2.left << " " << p2.keep << " " << p2.right << std::endl;
    std::cout << p3.left << " " << p3.keep << " " << p3.right << std::endl;
    std::cout << p4.left << " " << p4.keep << " " << p4.right << std::endl;
}

