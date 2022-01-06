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
//===========================================================================

void twoBlocks(){
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("s3.g");

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(4., 40, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "obj1"}, OT_sos, {1e2});
  komo.addObjective({1.}, FS_scalarProductXZ, {"R_gripperCenter", "obj1"}, OT_eq, {1e2}, {0.});

  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({2.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);

  arr basePos = C["obj1"]->getPosition();
  if (basePos.N != 3) return;
  basePos(2) += 0.3;
  komo.addObjective({2.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e2}, basePos);

  //run the solver
  komo.optimize();
  
  komo.view(true, "optimized configuration"); //display it
  while(komo.view_play(true));

  rai::wait();

}

std::shared_ptr<rai::Simulation> InitSimulation(rai::Configuration& RealWorld, std::vector<std::string> scenarioPaths) {
  for (std::string path : scenarioPaths) {
    RealWorld.addFile(path.c_str());
  }

  return std::make_shared<rai::Simulation>(RealWorld, rai::Simulation::SimulatorEngine::_bullet, 2);
}

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

void ExecuteKomoInSimulation(std::shared_ptr<KOMO> komo, std::shared_ptr<rai::Simulation> sim, double duration = 4) {
    arr next = komo.get()->getPath_qOrg();
    sim.get()->setMoveTo(next, duration); //append a 2 seconds spline -- this call is non-blocking!

    do
    {
        double tau=.001; //can set anything here...
        sim.get()->step({}, tau); //this executes the send spline while iterating physics with fine time resolution
        rai::wait(tau);
    }
    while (sim.get()->getTimeToMove() >= 0);
}

void ExecuteKomoInBotop(std::shared_ptr<KOMO> komo, rai::Configuration& c, std::shared_ptr<BotOp> bot, double duration = 4) {
    arr path = komo.get()->getPath_qOrg();
    bot.get()->moveLeap(path, duration); //append a 2 seconds spline -- this call is non-blocking!

    for(double speed=1.;speed<=5.;speed+=.5)
    {
        bot.get()->move(path, ARR(10.)/speed);
        
        Metronome tic(.01);
        while(bot.get()->step(c, -1.))
        {
            tic.waitForTic();
        }
    }
    return;
}

std::shared_ptr<KOMO> GetKomoToBlockTwo(const rai::Configuration& config) {
    std::shared_ptr<KOMO> komo = PrepareKomo(config);
    std::string blockName = "obj1";
    std::string blockName2 = "obj2";

    //task objectives:
    komo.get()->addObjective({1.},     FS_insideBox,       {"L_gripperCenter", blockName.c_str()},  OT_ineq, {1e2}, {});
    komo.get()->addObjective({1.},     FS_scalarProductXX, {"L_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductZZ, {"L_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductXZ, {"L_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"L_finger1",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"L_finger2",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"L_finger1",       blockName2.c_str()}, OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"L_finger2",       blockName2.c_str()}, OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({1.},     FS_qItself,         {},                           OT_eq,   {1e2}, {},     1);
    komo.get()->addObjective({1.},     FS_positionRel,     {"L_gripperCenter", blockName.c_str()},  OT_sos, {{1,3},{0,1e2,0}});

    komo.get()->optimize();
    return komo;
}

std::shared_ptr<KOMO> GetKomoUp(const rai::Configuration& config, double height = 0.3) {
    std::shared_ptr<KOMO> komo = PrepareKomo(config);
    const std::string blockName = "obj1";

    //task objectives:
    arr basePos = config[blockName.c_str()]->getPosition();
    basePos(2) += height;
    komo.get()->addObjective({1.}, FS_position, {"L_gripperCenter"}, OT_eq, {1e2}, basePos);
    komo.get()->addObjective({0., 1.}, FS_scalarProductXX, {"L_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
    komo.get()->addObjective({0., 1.}, FS_scalarProductYY, {"L_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
    komo.get()->addObjective({0., 1.}, FS_scalarProductZZ, {"L_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);

    komo.get()->optimize();
    return komo;
}

void flyingBlockSim() {
  rai::Configuration config;
  std::vector<std::string> scenarioPaths;
  scenarioPaths.emplace_back("./s4.g");
  std::shared_ptr<rai::Simulation> sim = InitSimulation(config, scenarioPaths);
  std::shared_ptr<KOMO> komo;

  komo = GetKomoToBlockTwo(config);
  ExecuteKomoInSimulation(komo, sim, 2.);
  
  // finger sollten das objekt erkennen und dann von alleine genau soweit schließen, bis kontakt entsteht
  sim.get()->closeGripper("R_gripper", 0.1, 1);
  while(sim.get()->getGripperIsGrasping("R_gripper"));
  // simu laufen lassen mit do step aus execute zb für 1 s
  komo = GetKomoUp(config);
  ExecuteKomoInSimulation(komo, sim, 0.5);

}

void flyingBlockBotop()
{
    rai::Configuration config;
    std::vector<std::string> scenarioPaths;
    scenarioPaths.emplace_back("../rai-robotModels/scenarios/pandasTable.g");
    std::shared_ptr<BotOp> bot = InitBotop(config, scenarioPaths);
    bot.get()->home(config);
    
    std::shared_ptr<KOMO> komo = GetKomoToBlockTwo(config);
    ExecuteKomoInBotop(komo, config, bot, 2.);
    bot.get()->gripperL->close();
    
    bot.get()->gripperL->close();
    while(!bot.get()->gripperL->isDone()) rai::wait(.1);
    
    komo = GetKomoUp(config);
    ExecuteKomoInBotop(komo, config, bot, 0.5);
    
    bot.get()->gripperL->open();


}

int main(int argc,char **argv){
    rai::initCmdLine(argc, argv);
    flyingBlockBotop();
    return 0;
}








// S.closeGripper("R_gripper", .05, .3);
