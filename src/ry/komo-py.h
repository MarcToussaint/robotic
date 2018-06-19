#pragma once

#include <KOMO/komo.h>

#include "types.h"

namespace ry{

  struct Configuration;

  struct KOMOpy_self : KOMO{
    Configuration* kin=0;
    bool denseMode = true;

    KOMOpy_self(Configuration* _kin, uint T);
    KOMOpy_self(Configuration* _kin, double phases, uint stepsPerPhase=20, double timePerPhase=5.);
    ~KOMOpy_self();

    void setDiscreteOpt(uint k);

    /// set an objective
    void setObjective(const arr& times, ObjectiveType type, const StringA& featureSymbols, const std::map<std::string, arr>& parameters={});

    /// output the defined problem as a generic graph, that can also be displayed, saved and loaded
    Graph getProblemGraph(bool includeValues=false);

    /// getting output after the optimization
    arr getPose(uint t, const rai::String& name);
    arr getRelPose(uint t, const rai::String& from, const rai::String& to);

  //  //TODO
  //  void setObjectivesFromGraph(const Graph& O);
  //  void setObjectivesFromString(istream& is); ///< first reads a generic graph, then interprets as objectives

  private:
    Task* setObjective(const arr& times, ObjectiveType type, TaskMap* feature, const arr& target=NoArr, double scale=1e1);
    TaskMap *symbols2feature(const StringA& featureSymbols, const std::map<std::string, arr> &parameters={});
  };

  struct KOMOpy{
    ptr<KOMOpy_self> self;

    KOMOpy(Configuration* _kin, uint T);
    KOMOpy(Configuration* _kin, double phases, uint stepsPerPhase, double timePerPhase);
    ~KOMOpy();

    void optimize(const Graph& features);
    void optimize2(const I_features& features);

    void adopt();
    void adopt(uint config);
    void adopt(double phase);
  };

}
