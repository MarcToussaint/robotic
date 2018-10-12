#include "lgp-py.h"

ry::LGPpy_self::LGPpy_self(ry::Configuration& _kin, const std::string& folFileName)
  : kin(_kin),
    K(kin.get()),
    L(FILE(folFileName.c_str())){

  initFolStateFromKin(L, K);

  LGP_Tree::init(K, L);
}

ry::LGPpy_self::~LGPpy_self(){
}

ry::LGPpy::LGPpy(Configuration& _kin, const std::string&  folFileName)
  : self(make_shared<ry::LGPpy_self>(_kin, folFileName)) {
}

ry::LGPpy::~LGPpy(){
}

void ry::LGPpy::optimizeFixedSequence(const std::string& seq){
  self->optFixedSequence(seq.c_str());
}

