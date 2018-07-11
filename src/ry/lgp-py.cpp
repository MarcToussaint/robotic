#include "lgp-py.h"
#include "configuration.h"

ry::LGPpy_self::LGPpy_self(ry::Configuration* _kin, const std::string& folFileName)
  : kin(_kin),
    K(_kin->K.get()),
    L(FILE(folFileName.c_str())){

  initFolStateFromKin(L, K);

  OptLGP::init(K, L);
}

ry::LGPpy_self::~LGPpy_self(){
}

ry::LGPpy::LGPpy(ry::Configuration* _kin, const std::string&  folFileName)
  : self(make_shared<ry::LGPpy_self>(_kin, folFileName)) {
}

ry::LGPpy::~LGPpy(){
}

void ry::LGPpy::optimizeFixedSequence(const std::string& seq){
  self->optFixedSequence(seq.c_str());
}

