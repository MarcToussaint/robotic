#pragma once

#include <vector>
#include <string>
#include <memory>

typedef std::vector<std::string> I_StringA;
template<class T> using ptr=std::shared_ptr<T>;
typedef std::map<std::string, std::string> I_dict;
typedef std::map<std::string, std::vector<double> > I_args;

typedef std::tuple<std::string, I_StringA, I_args> I_feature;
typedef std::vector<I_feature> I_features;


inline StringA I_conv(const I_StringA& x){
  StringA y(x.size());
  for(uint i=0;i<y.N;i++) y(i) = x[i];
  return y;
}

inline I_StringA I_conv(const StringA& x){
  I_StringA y;
  for(const rai::String& s:x) y.push_back(s.p);
  return y;
}

inline Graph I_conv(const I_dict& x){
  return Graph(x);
}
