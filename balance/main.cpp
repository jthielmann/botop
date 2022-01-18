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

#include "jt_base.h"

std::shared_ptr<KOMO> MoveArmAboveTheOther(rai::Configuration& config){
  std::shared_ptr<KOMO> komo = PrepareKomo(config, 40u, 1., 1.);

  komo.get()->addObjective({1.}, FS_positionRel, {"r_gripperCenter", "ref"}, OT_eq, {1e2}, {0., 0., 0.025});
  komo.get()->addObjective({1.},  FS_scalarProductZZ, {"obj1", "ref"}, OT_eq,   {1e2}, {1.});
  komo.get()->addObjective({1.},  FS_scalarProductZZ, {"ref", "world"}, OT_eq,   {1e2}, {1.});

  arr basePos = config["l_panda_coll6"]->getPosition();
  komo.get()->addObjective({0., 1.}, FS_position, {"l_panda_coll6"}, OT_eq, {1e2}, basePos);


  komo.get()->addObjective({0., .9},  FS_distance,     {"r_gripper", "l_panda_coll6"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger1", "l_panda_coll6"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger2", "l_panda_coll6"}, OT_ineq,   {1e2}, {-.01});
  
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_gripper", "l_panda_coll7"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger1", "l_panda_coll7"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger2", "l_panda_coll7"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"r_gripper", "l_gripper"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger1", "l_gripper"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger2", "l_gripper"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"r_gripper", "l_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger1", "l_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger2", "l_finger1"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"r_gripper", "l_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger1", "l_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"r_finger2", "l_finger2"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "l_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "l_finger1"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "l_finger1"}, OT_ineq,   {1e2}, {-.01});

  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "l_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "l_finger2"}, OT_ineq,   {1e2}, {-.01});
  komo.get()->addObjective({0., .9},  FS_distance,     {"obj1", "l_finger2"}, OT_ineq,   {1e2}, {-.01});

  //komo.get()->addObjective({0., 1.},  FS_vectorZ, {"ref"}, OT_eq,   {1e2}, {0., 0., 0.}, 1);

  komo.get()->addObjective({0.9, 1.}, FS_positionRel,     {"r_gripperCenter", "ref"}, OT_eq,   {1e2}, {0.,0.,0.2}, 2);
  komo.get()->addObjective({1.},      FS_qItself,         {},                           OT_eq,   {1e2}, {},     1);
  komo.get()->optimize();
  return komo;
}

void BalanceBlockOnArmBotop()
{
    rai::Configuration config;
    std::vector<std::string> scenarioPaths;
    scenarioPaths.emplace_back("s5.g");
    std::shared_ptr<BotOp> bot = InitBotop(config, scenarioPaths);
    bot.get()->home(config);
    
    std::shared_ptr<KOMO> komo = GetKomoToBlock(config);
    ExecuteKomoInBotop(komo, config, bot, 2.);
    
    bot.get()->gripperL->close();
    while(!bot.get()->gripperL->isDone()) rai::wait(.1);
    
    komo = MoveArmAboveTheOther(config);
    ExecuteKomoInBotop(komo, config, bot, 2.);
    
    bot.get()->gripperL->open();
}

int main(int argc,char **argv){
    rai::initCmdLine(argc, argv);
    BalanceBlockOnArmBotop();
    return 0;
}
