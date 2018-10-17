#pragma once

#include <LGP/LGP_tree.h>
#include <Core/thread.h>

#include "types.h"

namespace ry{

  typedef Var<rai::KinematicWorld> Config;

  struct LGPpy_self : LGP_Tree{
    Config& kin;
    rai::KinematicWorld K;
    FOL_World L;

    LGPpy_self(Config& _kin, const std::string& folFileName);
    ~LGPpy_self();
  };

  struct LGPpy{
    ptr<LGPpy_self> self;

    LGPpy(Config& _kin, const std::string& folFileName);
    ~LGPpy();

    void optimizeFixedSequence(const std::string& seq);
  };

}
