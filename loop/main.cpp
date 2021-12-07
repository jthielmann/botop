#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  BotOp bot(C, !rai::checkParameter<bool>("sim"));

  if(rai::checkParameter<bool>("loop")){
    arr q=bot.qHome;
    bot.moveLeap(q, 1.);
    while(bot.step(C));

    C.setJointState(bot.qHome);
    arr path = getLoopPath(C);
    bot.move(path, {5.});

    while(bot.step(C));
  }

  cout <<"bye bye" <<endl;

  return 0;
}
