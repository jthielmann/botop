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

void twoSticks(){
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame* obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  rai::Frame* obj2 = C.addFrame("object2");
  obj2->setPosition({.5, 0., 1.5});
  obj2->setQuaternion({1., 0., 1., 0});
  obj2->setShape(rai::ST_capsule, {.2, .02});
  obj2->setColor({1., .0, 1.});

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(3., 40, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2});
  komo.addObjective({2.}, FS_positionDiff, {"R_gripperCenter", "object2"}, OT_sos, {1e2});
  
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({2.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);

  //run the solver
  komo.optimize();

  komo.view(true, "optimized configuration"); //display it
  komo.view_play();

  rai::wait();

}


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

/*
void grabBlockByName(std::string armName, std::string blockName, std::vector<std::string> scenarioPaths, std::function<void(KOMO&, std::string&, std::string&)> komoInitCallback = defaultObjectives){
    //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration RealWorld;
  std::shared_ptr<rai::Simulation> S = InitSimulation(RealWorld, scenarioPaths);

  //-- optimize a single configuration using KOMO
  KOMO komo;                     //create a solver
  komo.setModel(RealWorld, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(2., 40, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!
  komoInitCallback(komo, armName, blockName);

  //example to get the joint vector from the optimized configuration
  arr q = komo.getConfiguration_qOrg(komo.T-1);
  RealWorld.setJointState(q); //set your working config into the optimized state

  komo.view(true, "optimized configuration"); //display it
  komo.view_play();
  S.get()->step();
  S.get()->setMoveTo(komo.getPath_qOrg(), 2.); //append a 2 seconds spline -- this call is non-blocking!

  for(uint t=0;;t++){
    //cout <<"time to move: " <<S.getTimeToMove() <<endl;
    arr currentPos = RealWorld[blockName.c_str()]->getPosition();
    double tau=.001; //can set anything here...
    S.get()->step({}, tau); //this executes the send spline while iterating physics with fine time resolution
    rai::wait(tau);
    if(S.get()->getTimeToMove()<0.) break; //this checks the end of the spline
  }

  rai::wait();
}
*/

void moveStick() {

  // pass per reference to functions
  rai::Configuration C;
  std::vector<std::string> scenarioPaths;
  scenarioPaths.emplace_back("../s3.g");
  std::shared_ptr<rai::Simulation> S = InitSimulation(C, scenarioPaths);

  KOMO komo;                     //create a solver
  
  // init für komo berechnung, hier ist der roboter
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  uint stepsPerPhase = 40;
  double phases = 1.;
  komo.setTiming(phases, stepsPerPhase, 5., 2);

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)
  std::string blockName = "obj1";
  
  //task objectives:
  komo.addObjective({1.}, FS_positionRel, {"R_gripperCenter", blockName.c_str()}, OT_sos, {{1,3},{0,1e2,0}});
  komo.addObjective({1.}, FS_insideBox,   {"R_gripperCenter", blockName.c_str()}, OT_ineq, {1e2}, {});

  //komo.addObjective({1.0, 2.0}, FS_distance, {"R_gripper", blockName.c_str()}, OT_ineq, {1e2}, {});
  //komo.addObjective({}, FS_distance, {"R_gripper", blockName.c_str()}, OT_ineq, {1e2}, {});
  komo.addObjective({1.}, FS_scalarProductXX, {"R_gripperCenter", blockName.c_str()}, OT_eq,   {1e2}, {0.});
  komo.addObjective({1.}, FS_scalarProductXZ, {"R_gripperCenter", blockName.c_str()}, OT_eq,   {1e2}, {0.});
  //komo.addObjective({1.}, FS_scalarProductYY, {"R_gripperCenter", blockName.c_str()}, OT_eq,   {1e2}, {0.});
  komo.addObjective({0., 1.}, FS_distance, {"R_finger1", blockName.c_str()}, OT_ineq, {1e2}, {.05});
  komo.addObjective({0., 1.}, FS_distance, {"R_finger2", blockName.c_str()}, OT_ineq, {1e2}, {.05});
  komo.optimize();
  arr q = komo.getConfiguration_qOrg(0);
  // return q (der berechnete pfad)

  C.setJointState(q);

  // move to the block
  arr next = komo.getConfiguration_qOrg(komo.T - (phases - 1) * stepsPerPhase - 1);
  rai::wait();
  S.get()->setMoveTo(next, 4.); //append a 2 seconds spline -- this call is non-blocking!
  do {
    double tau=.001; //can set anything here...
    S.get()->step({}, tau); //this executes the send spline while iterating physics with fine time resolution
    rai::wait(tau);
  }
  while (S.get()->getTimeToMove() >= 0);

  rai::wait();

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
   
  do {
    double tau=.001; //can set anything here...
    sim.get()->step({}, tau); //this executes the send spline while iterating physics with fine time resolution
    rai::wait(tau);
  }
  while (sim.get()->getTimeToMove() >= 0);
}

std::shared_ptr<KOMO> GetKomoToBlock(const rai::Configuration& config) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  std::string blockName = "obj1";
  
  komo.get()->addObjective({1.},     FS_positionRel,     {"R_gripperCenter", blockName.c_str()}, OT_sos, {{1,3},{0,1e2,0}});
  //task objectives:
  komo.get()->addObjective({1.},     FS_insideBox,       {"R_gripperCenter", blockName.c_str()}, OT_ineq, {1e2}, {});
  komo.get()->addObjective({1.},     FS_scalarProductXX, {"R_gripperCenter", blockName.c_str()}, OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({1.},     FS_scalarProductZZ, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({1.},     FS_scalarProductXZ, {"R_gripperCenter", blockName.c_str()}, OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger1",       blockName.c_str()}, OT_ineq, {1e2}, {-.01});
  komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger2",       blockName.c_str()}, OT_ineq, {1e2}, {-.01});


  komo.get()->optimize();
  return komo;
}

std::shared_ptr<KOMO> GetKomoToBlockTwo(const rai::Configuration& config) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  std::string blockName = "obj1";
  std::string blockName2 = "obj2";
  
  //task objectives:
  
  komo.get()->addObjective({1.},     FS_insideBox,       {"R_gripperCenter", blockName.c_str()},  OT_ineq, {1e2}, {});
  komo.get()->addObjective({1.},     FS_scalarProductXX, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({1.},     FS_scalarProductZZ, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({1.},     FS_scalarProductXZ, {"R_gripperCenter", blockName.c_str()},  OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger1",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
  komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger2",       blockName.c_str()},  OT_ineq, {1e2}, {-.01});
  komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger1",       blockName2.c_str()}, OT_ineq, {1e2}, {-.01});
  komo.get()->addObjective({0., 1.}, FS_distance,        {"R_finger2",       blockName2.c_str()}, OT_ineq, {1e2}, {-.01});
  komo.get()->addObjective({1.},      FS_qItself,         {},                           OT_eq,   {1e2}, {},     1);
  komo.get()->addObjective({1.},     FS_positionRel,     {"R_gripperCenter", blockName.c_str()},  OT_sos, {{1,3},{0,1e2,0}});

  komo.get()->optimize();
  return komo;
}

std::shared_ptr<KOMO> GetKomoUp(const rai::Configuration& config, double height = 0.3) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  const std::string blockName = "obj1";
  
  //task objectives:
  arr basePos = config[blockName.c_str()]->getPosition();
  basePos(2) += height;
  komo.get()->addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e2}, basePos);
  komo.get()->addObjective({0., 1.}, FS_scalarProductXX, {"R_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
  komo.get()->addObjective({0., 1.}, FS_scalarProductYY, {"R_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);
  komo.get()->addObjective({0., 1.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq,   {1e2}, {0.}, 1);

  komo.get()->optimize();
  return komo;
}


std::shared_ptr<KOMO> GetKomoToBall(const rai::Configuration& config) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  
  const std::string object = "ball";
  //task objectives:
  arr basePos = config[object.c_str()]->getPosition();
  basePos(2) += 0.6;
  komo.get()->addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e2}, basePos);
  //komo.get()->addObjective({1.}, FS_scalarProductXX, {"R_gripperCenter", object.c_str()}, OT_eq,   {1e2}, {0.});
  komo.get()->addObjective({1.}, FS_scalarProductXX, {"R_gripperCenter", object.c_str()}, OT_eq,   {1e2}, {-1.});

  komo.get()->optimize();
  return komo;
}

std::shared_ptr<KOMO> GetKomoToFinish(rai::Configuration& config) {
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  komo.get()->addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.get()->optimize();
  return komo;
}

void SimWaitForSeconds(double seconds, std::shared_ptr<rai::Simulation> sim){
  double tau = .01;
  for(double i = 0; i < seconds; i+=tau) {
    sim.get()->step({}, tau);
    rai::wait(tau);
  }
}

void stickAndBlock() {
  rai::Configuration config;
  std::vector<std::string> scenarioPaths;
  scenarioPaths.emplace_back("./s3.g");
  std::shared_ptr<rai::Simulation> sim = InitSimulation(config, scenarioPaths);
  
  std::shared_ptr<KOMO> komo;
  //komo = GetKomoUp(config);
  //ExecuteKomoInSimulation(komo, sim, 2.);

  komo = GetKomoToBlock(config);
  ExecuteKomoInSimulation(komo, sim, 2.);
  rai::wait();
  // finger sollten das objekt erkennen und dann von alleine genau soweit schließen, bis kontakt entsteht
  sim.get()->closeGripper("R_gripper", 0.1, 1);
  while(sim.get()->getGripperIsGrasping("R_gripper"));
  
  // simu laufen lassen mit do step aus execute zb für 1 s
  komo = GetKomoUp(config);
  ExecuteKomoInSimulation(komo, sim, 2.);
  
  // braucht gewicht
  komo = GetKomoToBall(config);
  ExecuteKomoInSimulation(komo, sim, 2.);

  sim.get()->openGripper("R_gripper", 1, 1);

  komo = GetKomoToFinish(config);
  ExecuteKomoInSimulation(komo, sim, 2);
  rai::wait();
}

void flyingBlock() {
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
  SimWaitForSeconds(1., sim);
  // simu laufen lassen mit do step aus execute zb für 1 s
  komo = GetKomoUp(config);
  ExecuteKomoInSimulation(komo, sim, 0.5);

  SimWaitForSeconds(1., sim);

}

std::shared_ptr<KOMO> StraightenSecondArm(rai::Configuration& config){
  std::shared_ptr<KOMO> komo = PrepareKomo(config);
  const std::string blockName = "obj1";
  
  //task objectives:
  arr basePos = config[blockName.c_str()]->getPosition();
  basePos(2) -= 0.4;

  komo.get()->addObjective({1.}, FS_position, {"L_gripperCenter"}, OT_eq, {1e2}, basePos);

  komo.get()->optimize();
  return komo;
}

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

void balanceBlockOnArm() {
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

void standUp() {
  // config stuff

  // move arms to shoes

  // grip

  // move shoes in position

  // stand up
  
}

void balanceBall() {
  // config stuff

  // arm to ball on plate

  // grip

  // move up

  // circle

  // pass plate to other arm
}



int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  //std::vector<std::string> scenarios = {"./s3.g"};
  //grabBlockByName(std::string("R_gripper"), std::string("obj1"), scenarios);


  //  using_KOMO_for_IK();
  //twoSticks();
  //twoBlocks();
  flyingBlock();
  //balanceBlockOnArm();
  //moveStick();
  return 0;
}







// S.closeGripper("R_gripper", .05, .3);