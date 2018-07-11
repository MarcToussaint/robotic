#pragma once

#include <LGP/optLGP.h>

#include "types.h"

namespace ry{

  struct Configuration;

  struct LGPpy_self : OptLGP{
    Configuration* kin=0;
    rai::KinematicWorld K;
    FOL_World L;

    LGPpy_self(Configuration* _kin, const std::string& folFileName);
    ~LGPpy_self();
  };

  struct LGPpy{
    ptr<LGPpy_self> self;

    LGPpy(Configuration* _kin, const std::string& folFileName);
    ~LGPpy();

    void optimizeFixedSequence(const std::string& seq);
  };

}
