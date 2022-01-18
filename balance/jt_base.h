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

std::shared_ptr<BotOp> InitBotop(rai::Configuration& RealWorld, std::vector<std::string> scenarioPaths);

std::shared_ptr<KOMO> PrepareKomo(const rai::Configuration& c, uint stepsPerPhase = 40, double phases = 1., double weight = 1.);

void ExecuteKomoInBotop(std::shared_ptr<KOMO> komo, rai::Configuration& c, std::shared_ptr<BotOp> bot, double duration = 10);

std::shared_ptr<KOMO> GetKomoToBlock(const rai::Configuration& config);

std::shared_ptr<KOMO> GetKomoUp(const rai::Configuration& config, double height = 0.3);