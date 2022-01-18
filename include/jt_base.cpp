#include <Geo/depth2PointCloud.h>

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>
#include <functional>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

std::shared_ptr<BotOp> InitBotop(rai::Configuration& RealWorld, std::vector<std::string> scenarioPaths) {
    for (std::string path : scenarioPaths) {
        RealWorld.addFile(path.c_str());
    }

    return std::make_shared<BotOp>(RealWorld, rai::checkParameter<bool>("real"));
}

std::shared_ptr<KOMO> PrepareKomo(const rai::Configuration& c, uint stepsPerPhase = 40, double phases = 1., double weight = 1.) {
  
    std::shared_ptr<KOMO> komo = std::make_shared<KOMO>();
    // init für komo berechnung, hier ist der roboter
    komo.get()->setModel(c, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)


    komo.get()->setTiming(phases, stepsPerPhase, 5., 2);
    //the default control objective:
    komo.get()->add_qControlObjective({}, 2, weight); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)
    return komo;
}

void ExecuteKomoInBotop(std::shared_ptr<KOMO> komo, rai::Configuration& c, std::shared_ptr<BotOp> bot, double duration = 10) {
    arr path = komo.get()->getPath_qOrg();
    bot.get()->move(path, ARR(duration));
    Metronome tic(.01);
    while(bot.get()->step(c, -1.))
    {
        tic.waitForTic();
    }
    return;
}

std::shared_ptr<KOMO> GetKomoToBlock(const rai::Configuration& config) {
    std::shared_ptr<KOMO> komo = PrepareKomo(config);
    std::string blockName = "obj1";

    //task objectives:
    komo.get()->addObjective({1.},     FS_insideBox,       {"l_gripperCenter", blockName.c_str()},  OT_ineq, {1e2}, {});
    komo.get()->addObjective({1.},     FS_scalarProductXX, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductZZ, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.7});
    komo.get()->addObjective({1.},     FS_scalarProductYY, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"l_finger1",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"l_finger2",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({1.},     FS_qItself,         {},                           OT_eq,   {1e2}, {},     1);
    komo.get()->addObjective({1.},     FS_positionRel,     {"l_gripperCenter", blockName.c_str()},  OT_sos, {{1,3},{0,1e2,0}});

    komo.get()->optimize();
    return komo;
}

std::shared_ptr<KOMO> GetKomoToStick(const rai::Configuration& config) {
    std::shared_ptr<KOMO> komo = PrepareKomo(config);
    std::string blockName = "obj1";

    //task objectives:
    komo.get()->addObjective({1.},     FS_insideBox,       {"l_gripperCenter", blockName.c_str()},  OT_ineq, {1e2}, {});
    komo.get()->addObjective({1.},     FS_scalarProductXX, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductZZ, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    //komo.get()->addObjective({1.},     FS_scalarProductXZ, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductYY, {"l_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"l_finger1",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"l_finger2",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({1.},     FS_qItself,         {},                           OT_eq,   {1e2}, {},     1);
    komo.get()->addObjective({1.},     FS_positionRel,     {"l_gripperCenter", blockName.c_str()},  OT_sos, {{1,3},{0,1e2,0}});

    komo.get()->optimize();
    return komo;
}

std::shared_ptr<KOMO> GetKomoUp(const rai::Configuration& config, double height = 0.3) {
    std::shared_ptr<KOMO> komo = PrepareKomo(config);
    const std::string blockName = "obj1";

    //task objectives:
    arr basePos = config[blockName.c_str()]->getPosition();
    basePos(2) += height;
    komo.get()->addObjective({1.},     FS_position,        {"l_gripperCenter"},          OT_eq,   {1e2}, basePos);
    komo.get()->addObjective({0., 1.}, FS_scalarProductXX, {"l_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
    komo.get()->addObjective({0., 1.}, FS_scalarProductYY, {"l_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
    komo.get()->addObjective({0., 1.}, FS_scalarProductZZ, {"l_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
    komo.get()->addObjective({1.},     FS_qItself,         {},                           OT_eq,   {1e2}, {},   1);

    komo.get()->optimize();
    return komo;
}

void simpleUp()
{
    rai::Configuration config;
    std::vector<std::string> scenarioPaths;
    scenarioPaths.emplace_back("pandasTableLocal.g");
    std::shared_ptr<BotOp> bot = InitBotop(config, scenarioPaths);

    bot.get()->home(config);

    std::shared_ptr<KOMO> komo = GetKomoToBlock(config);
    ExecuteKomoInBotop(komo, config, bot, 10.);

    bot.get()->gripperL->close();
    while(!bot.get()->gripperL->isDone()) rai::wait(.1);

    komo = GetKomoUp(config);
    ExecuteKomoInBotop(komo, config, bot, 2.);
    
}
