#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <Core/array.h>
#include <Core/graph.h>

namespace ry{

typedef std::pair<std::vector<unsigned int>, std::vector<double> > I_arr;
typedef std::vector<std::string> I_StringA;
template<class T> using ptr=std::shared_ptr<T>;
typedef std::map<std::string, std::string> I_dict;
typedef std::map<std::string, std::vector<double> > I_args;

typedef std::tuple<std::vector<double>, I_StringA, I_args> I_feature;
typedef std::vector<I_feature> I_features;

typedef std::tuple<std::vector<double>, std::string, I_StringA, I_args> I_objective;
typedef std::vector<I_objective> I_objectives;

struct FrameInfo{
  int ID;
  std::string name;
  std::string parent;
  I_StringA children;
  std::vector<double> X;
  std::vector<double> Q;
};

}

inline StringA I_conv(const ry::I_StringA& x){
  StringA y(x.size());
  for(uint i=0;i<y.N;i++) y(i) = x[i];
  return y;
}

inline ry::I_StringA I_conv(const StringA& x){
  ry::I_StringA y;
  for(const rai::String& s:x) y.push_back(s.p);
  return y;
}

inline Graph I_conv(const ry::I_dict& x){
  return Graph(x);
}

inline ry::I_arr I_conv(const arr& x){
  ry::I_arr y;
  y.first = x.dim();
  y.second = x;
  return y;
}

inline arr I_conv(const ry::I_arr& x){
  arr y;
  y = conv_stdvec2arr(x.second);
  y.reshape(conv_stdvec2arr(x.first));
  return y;
}
