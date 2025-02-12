#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  BotOp bot(C, rai::checkParameter<bool>("real"));

  if(rai::checkParameter<bool>("lurd")){
    arr q=bot.qHome;
    q(0) += .5;
    bot.moveLeap(q, 1.);
    while(bot.step(C));

    q=bot.qHome;
    q(1) -= .5;
    bot.moveLeap(q, 1.);
    while(bot.step(C));

    q(0) -= .5;
    bot.moveLeap(q, 1.);
    while(bot.step(C));

    q(1) += .5;
    bot.moveLeap(q, 1.);
    while(bot.step(C));
  }

<<<<<<< HEAD
  cout <<"bye bye" <<endl;
=======
  if (rai::checkParameter<bool>("test")){

      bot.gripperL->close();
      while(!bot.gripperL->isDone()) rai::wait(.1);

      bot.gripperL->open();
      while(!bot.gripperL->isDone()) rai::wait(.1);
  }

  cout <<"bye bye" <<endl;
  rai::wait(.1);
>>>>>>> abfe4326d461022abb08cd47da3ec2b5cd1fcb27

  return 0;
}
