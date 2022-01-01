#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>
#include <functional>

//===========================================================================
std::shared_ptr<KOMO> MoveArmAboveTheOther(rai::Configuration& config){
  std::shared_ptr<KOMO> komo = PrepareKomo(config, 40u, 1., 1.);

  komo.get()->addObjective({1.}, FS_positionRel, {"R_gripperCenter", "ref"}, OT_eq, {1e2}, {0., 0., 0.025});
  komo.get()->addObjective({1.},  FS_scalarProductZZ, {"obj1", "ref"}, OT_eq,   {1e2}, {1.});
  komo.get()->addObjective({1.},  FS_scalarProductZZ, {"ref", "world"}, OT_eq,   {1e2}, {1.});

  arr basePos = config["L_panda_coll6"]->getPosition();
  komo.get()->addObjective({0., 1.}, FS_position, {"L_panda_coll6"}, OT_eq, {1e2}, basePos);


  komo.get()->addObjective({0., .9},  FS_distance,     {"R_gripper", "L_panda_coll6"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger1", "L_panda_coll6"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger2", "L_panda_coll6"}, OT_ineq,   {1e2}, {-.01});
  
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_gripper", "L_panda_coll7"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger1", "L_panda_coll7"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger2", "L_panda_coll7"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"R_gripper", "L_gripper"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger1", "L_gripper"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger2", "L_gripper"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"R_gripper", "L_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger1", "L_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger2", "L_finger1"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"R_gripper", "L_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger1", "L_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"R_finger2", "L_finger2"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "L_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "L_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "L_finger1"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "L_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "L_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "L_finger2"}, OT_ineq,   {1e2}, {-.01});

  //komo.get()->addObjective({0., 1.},  FS_vectorZ, {"ref"}, OT_eq,   {1e2}, {0., 0., 0.}, 1);

  komo.get()->addObjective({0.9, 1.}, FS_positionRel,     {"R_gripperCenter", "ref"}, OT_eq,   {1e2}, {0.,0.,0.2}, 2);
  komo.get()->addObjective({1.},      FS_qItself,         {},                           OT_eq,   {1e2}, {},     1);
  komo.get()->optimize();
  return komo;
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

void ExecuteKomoInBotop(std::shared_ptr<KOMO> komo, std::shared_ptr<BotOp> bot, double duration = 4) {
    arr path = komo.get()->getPath_qOrg();
    bot.get()->setMoveTo(next, duration); //append a 2 seconds spline -- this call is non-blocking!

    for(double speed=1.;speed<=5.;speed+=.5)
    {
        bot.move(path, ARR(10.)/speed);
        
        Metronome tic(.01);
        while(bot.step(C, -1.))
        {
            tic.waitForTic();
        }
    }
    return;
}

std::shared_ptr<KOMO> GetKomoToBlock(const rai::Configuration& config) {
    std::shared_ptr<KOMO> komo = PrepareKomo(config);
    std::string blockName = "obj1";
    
    //task objectives:
    komo.get()->addObjective({1.},     FS_insideBox,       {"R_gripperCenter", blockName.c_str()},  OT_ineq, {1e2}, {});
    komo.get()->addObjective({1.},     FS_scalarProductXX, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductZZ, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({1.},     FS_scalarProductXZ, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger1",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger2",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
    komo.get()->addObjective({1.},     FS_qItself,         {},                                      OT_eq,   {1e2}, {},     1);
    komo.get()->addObjective({1.},     FS_positionRel,     {"R_gripperCenter", blockName.c_str()},  OT_sos,  {{1,3},{0,1e2,0}});

    komo.get()->optimize();
    return komo;
}

void BalanceBlockOnArmSim() {
  rai::Configuration config;
  std::vector<std::string> scenarioPaths;
  scenarioPaths.emplace_back("./s5.g");
  std::shared_ptr<rai::Simulation> sim = InitSimulation(config, scenarioPaths);
  std::shared_ptr<KOMO> komo;
  
  komo = GetKomoToBlock(config);
  ExecuteKomoInSimulation(komo, sim, 2);
  // finger sollten das objekt erkennen und dann von alleine genau soweit schließen, bis kontakt entsteht
  sim.get()->closeGripper("R_gripper", 0.1, 1);
  //while(sim.get()->getGripperIsGrasping("R_gripper"));
  SimWaitForSeconds(.5, sim);

  komo = MoveArmAboveTheOther(config);
  ExecuteKomoInSimulation(komo, sim, 2.);

  sim.get()->openGripper("R_gripper", 0.1, 1);
  SimWaitForSeconds(2., sim);

  rai::wait();
}

void BalanceBlockOnArmBotop()
{
    rai::Configuration config;
    std::vector<std::string> scenarioPaths;
    scenarioPaths.emplace_back("../rai-robotModels/scenarios/pandasTable.g");
    std::shared_ptr<BotOp> bot = InitBotop(config, scenarioPaths);
    bot.get->home(C);
    
    std::shared_ptr<KOMO> komo = GetKomoToBlock(config);
    ExecuteKomoInBotop(komo, sim, 2.);
    
    bot.get->gripperL->close();
    while(!bot.gripperL->isDone()) rai::wait(.1);
    
    komo = MoveArmAboveTheOtherBotop(config);
    ExecuteKomoInBotop(komo, sim, 0.5);
    
    bot.gripperL->open();
}

int main(int argc,char **argv){
    rai::initCmdLine(argc, argv);
    BalanceBlockOnArmBotop();
    return 0;
}
