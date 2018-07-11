#include "lgp-py.h"
#include "configuration.h"

ry::LGPpy_self::LGPpy_self(ry::Configuration* _kin)
  : kin(_kin),
    K(_kin->K.get()),
    L(FILE("fol.g")){

  initFolStateFromKin(L, K);

  OptLGP::init(K, L);
}

ry::LGPpy_self::~LGPpy_self(){
}

ry::LGPpy::LGPpy(ry::Configuration* _kin)
  : self(make_shared<ry::LGPpy_self>(_kin)) {
}

ry::LGPpy::~LGPpy(){
}

void ry::LGPpy::optimizeFixedSequence(const rai::String& seq){
  self->optFixedSequence(seq);
}

