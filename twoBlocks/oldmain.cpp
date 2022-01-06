#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================s

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

  // move up
  q(1) += .5;
  bot.moveLeap(q, 1.);
  while(bot.step(C));




  cout <<"bye bye" <<endl;

  return 0;
}
