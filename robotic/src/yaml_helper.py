#!/usr/bin/env python3

import sys
import yaml
#from ruamel import yaml

# yaml dump tweaks
class noflow_dict(dict):
    pass
def noflow_dict_rep(dumper, data):
    return dumper.represent_mapping("tag:yaml.org,2002:map", data, flow_style=False)
yaml.add_representer(noflow_dict, noflow_dict_rep)

class quoted_string(str):
    pass
def quoted_string_rep(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style='"')
yaml.add_representer(quoted_string, quoted_string_rep)

def yaml_write_dict(data, filename):
    with open(filename, 'w') as fil:
        yaml.dump(noflow_dict(data), fil, default_flow_style=True, sort_keys=False, width=500)

if __name__ == "__main__":
    filename = sys.argv[1]
    with open(filename, 'r', encoding='utf-8') as fil:
        data = yaml.safe_load(fil)
    print(yaml.dump(noflow_dict(data), None, default_flow_style=True, sort_keys=False, width=500))
        
