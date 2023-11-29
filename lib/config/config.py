from .yacs import CfgNode as CN
import carla
import argparse
from . import yacs
import os

cfg = CN()

cfg.save_tag = 'default'
# recorder
cfg.record_dir = 'data/record'
# result
cfg.result_dir = 'data/result'

def parse_cfg(cfg):
    if len(cfg.task) == 0:
        raise ValueError('task must be specified')

    print('EXP NAME: ', cfg.exp_name)
    cfg.record_dir = os.path.join(cfg.record_dir, cfg.task, cfg.exp_name, cfg.map)
    cfg.result_dir = os.path.join(cfg.result_dir, cfg.task, cfg.exp_name, cfg.map, cfg.save_tag)
    modules = [key for key in cfg if '_module' in key]
    for module in modules:
        cfg[module.replace('_module', '_path')] = cfg[module].replace('.', '/') + '.py'

def make_cfg(args):
    def merge_cfg(cfg_file, cfg):
        with open(cfg_file, 'r') as f:
            current_cfg = yacs.load_cfg(f)
        if 'parent_cfg' in current_cfg.keys():
            cfg = merge_cfg(current_cfg.parent_cfg, cfg)
            cfg.merge_from_other_cfg(current_cfg)
        else:
            cfg.merge_from_other_cfg(current_cfg)
        return cfg
    cfg_ = merge_cfg(args.cfg_file, cfg)
    parse_cfg(cfg_)
    return cfg_

def config_to_trans(trans_config):
    transform = carla.Transform(carla.Location(trans_config["location"][0],
                                               trans_config["location"][1],
                                               trans_config["location"][2]),
                                carla.Rotation(trans_config["rotation"][0],
                                               trans_config["rotation"][1],
                                               trans_config["rotation"][2]))
    return transform

parser = argparse.ArgumentParser()
parser.add_argument("--cfg_file", default="configs/base.yaml", type=str)
parser.add_argument('--random_seed', help='Set seed for repeating executions (default: None)', default=None)
args = parser.parse_args()

cfg = make_cfg(args)
print(cfg)
