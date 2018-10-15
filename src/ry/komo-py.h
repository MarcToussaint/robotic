#pragma once

#include "types.h"

#include <Core/thread.h>
#include <KOMO/komo.h>
#include <pybind11/numpy.h>

namespace ry{

  typedef Var<rai::KinematicWorld> Config;

  struct KOMOpy{
    Var<rai::KinematicWorld> kin;
    ptr<KOMO> self;

    KOMOpy(Config& _kin, uint T);
    KOMOpy(Config& _kin, double phases, uint stepsPerPhase, double timePerPhase);
    ~KOMOpy();

    //--
    void makeObjectsFree(const I_StringA& objs);
    void activateCollisionPairs(const std::vector<std::pair<std::string, std::string>>& collision_pairs);
    void deactivateCollisionPairs(const std::vector<std::pair<std::string, std::string>>& collision_pairs);
    void timeOptimization();

    void clearObjectives();

    //-- core methods to add objectives
    void addObjective2(const std::vector<int>& timeInterval, const ObjectiveType& type, const FeatureSymbol& feature, const I_StringA& frames={}, const std::vector<double>& scale={}, const std::vector<double>& target={}, int order=-1);
    void addObjective(const std::vector<double>& timeInterval, const ObjectiveType& type, const FeatureSymbol& feature, const I_StringA& frames={}, const std::vector<double>& scale={}, const std::vector<double>& target={}, int order=-1);

    //-- standard motion problems
    void setMotionTo(const arr &q, const StringA& joints, const char* endeff, double up, double down);

    //-- macros
    void add_grasp(int conf, const char* gripper, const char* object);
    void add_place(int conf, const char* object, const char* table);
    void add_StableRelativePose(const std::vector<int>& confs, const char* gripper, const char* object);
    void add_StablePose(const std::vector<int>& confs, const char* object);
    void add_resting(int conf1, int conf2, const char* object);
    void add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper);

    //-- basis of LGP
//    void addSkeleton(const std::vector<double>& times, I_StringA symbols);
//    void setSkeleton();
//    void skeleton2bound(bool collision=true);

    //-- optimize
    void optimize();

    KOMO* operator->() { return &*self; }
    KOMO* operator()() { return &*self; }

    //-- get results
    int getT();
    void getConfiguration(int t);
//    std::string getReport();
    Graph getProblemGraph();
    double getConstraintViolations();
    double getCosts();

    //-- display
    void display();
    void validate();
  };

}
