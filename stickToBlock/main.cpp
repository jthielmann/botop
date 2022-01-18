#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>
#include <functional>

#include "../include/jt_base.h"

std::shared_ptr<KOMO> GetKomoToBall(const rai::Configuration& config) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  
  const std::string object = "ball";
  //task objectives:
  arr basePos = config[object.c_str()]->getPosition();
  basePos(2) += 0.6;
  komo.get()->addObjective({1.}, FS_position, {"l_gripperCenter"}, OT_eq, {1e2}, basePos);


  komo.get()->addObjective({1.}, FS_scalarProductZZ, {object.c_str(), "l_gripperCenter"}, OT_eq,   {1e2}, {.7});
  //komo.get()->addObjective({1.}, FS_scalarProductZZ, {"l_gripperCenter", object.c_str()}, OT_eq,   {1e2}, {0.});
  //komo.get()->addObjective({1.}, FS_scalarProductXY, {"l_gripperCenter", object.c_str()}, OT_eq,   {1e2}, {1.});
  komo.get()->addObjective({1.}, FS_scalarProductXY, {"l_gripperCenter", object.c_str()}, OT_eq,   {1e2}, {-1.});



  komo.get()->optimize();
  return komo;
}

std::shared_ptr<KOMO> GetKomoToFinish(rai::Configuration& config) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  komo.get()->addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.get()->optimize();
  return komo;
}

void stickAndBlockBotop() {
    
    rai::Configuration config;
    std::vector<std::string> scenarioPaths;
    scenarioPaths.emplace_back("pandasTableLocal.g");

    std::shared_ptr<BotOp> bot = InitBotop(config, scenarioPaths);
    bot.get()->home(config);
    std::shared_ptr<KOMO> komo;
    rai::wait();
    komo = GetKomoToBlock(config);
    ExecuteKomoInBotop(komo, config, bot, 5.);

    bot.get()->gripperL->close();
    while(!bot.get()->gripperL->isDone()) rai::wait(.1);

    // simu laufen lassen mit do step aus execute zb für 1 s
    komo = GetKomoToBall(config);
    ExecuteKomoInBotop(komo, config, bot, 5.);
    rai::wait();
/*    komo = GetKomoToFinish(config);
    ExecuteKomoInBotop(komo, config, bot, 1.);
    rai::wait();
    */
}

int main(int argc,char **argv)
{
    rai::initCmdLine(argc, argv);
    stickAndBlockBotop();
    return 0;
}
