#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  BotOp bot(C, rai::checkParameter<bool>("real"));

  // move to base position
  arr q=bot.qHome;
  bot.moveLeap(q, 1.);
  while(bot.step(C));

  // move to block
  // TODO
/*
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "obj1"}, OT_sos, {1e2});
  komo.addObjective({1.}, FS_scalarProductXZ, {"R_gripperCenter", "obj1"}, OT_eq, {1e2}, {0.});
*/
  // close gripper
  bot.gripperL->close();
  while(!bot.gripperL->isDone()) rai::wait(.1);

  // move arm above the other
  /*
  TODO

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

  */

  // open gripper
  bot.gripperL->open();
  while(!bot.gripperL->isDone()) rai::wait(.1);


  cout <<"bye bye" <<endl;

  return 0;
}
