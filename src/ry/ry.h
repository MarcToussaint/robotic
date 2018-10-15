#include <Kin/kin.h>
#include <Core/thread.h>
#include <Kin/kinViewer.h>

#include "lgp-py.h"

namespace ry{

  typedef Var<rai::KinematicWorld> Config;

  struct ConfigView { shared_ptr<KinViewer> view; };

  struct RyKOMO{ shared_ptr<KOMO> komo; };

  struct RyFeature { Feature *feature=0; };
}
