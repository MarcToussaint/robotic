#pragma once

#include <LGP/LGP_tree.h>
#include <Core/thread.h>

#include "types.h"

namespace ry{

  typedef Var<rai::KinematicWorld> Configuration;

  struct LGPpy_self : LGP_Tree{
    Configuration& kin;
    rai::KinematicWorld K;
    FOL_World L;

    LGPpy_self(Configuration& _kin, const std::string& folFileName);
    ~LGPpy_self();
  };

  struct LGPpy{
    ptr<LGPpy_self> self;

    LGPpy(Configuration& _kin, const std::string& folFileName);
    ~LGPpy();

    void optimizeFixedSequence(const std::string& seq);
  };

}
