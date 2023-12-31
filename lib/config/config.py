from .yacs import CfgNode as CN
import carla
import argparse
from . import yacs
import os

cfg = CN()

# recorder
cfg.record_dir = 'data/record'
# result
cfg.result_dir = 'data/result'

def parse_cfg(cfg, args):
    if len(cfg.task) == 0:
        raise ValueError('task must be specified')

    print('EXP NAME: ', cfg.exp_name)
    if args.seed is not None:
        cfg.seed = args.seed
    if args.map is not None:
        cfg.map = args.map
    if args.num_vehicles is not None:
        cfg.carla.num_of_npc_vehicles = args.num_vehicles
    if args.frame is not None:
        cfg.all_frame_iter = args.frame
    cfg.record_dir = os.path.abspath(os.path.join(cfg.record_dir, cfg.task, cfg.exp_name, cfg.map))
    cfg.result_dir = os.path.abspath(os.path.join(cfg.result_dir, cfg.task, cfg.exp_name, cfg.map))
    if args.output is not None:
        cfg.result_dir = os.path.abspath(os.path.join(args.output, f"{cfg.map}_{cfg.carla.num_of_npc_vehicles:04d}_{cfg.seed:04d}/renderings"))
    if not os.path.exists(cfg.record_dir):
        os.makedirs(cfg.record_dir)
    if not os.path.exists(cfg.result_dir):
        os.makedirs(cfg.result_dir)
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
    parse_cfg(cfg_, args)
    return cfg_

parser = argparse.ArgumentParser()
parser.add_argument("--cfg_file", default="configs/basic.yaml", type=str)
parser.add_argument('--seed', help='Set seed for repeating executions (default: None)', type=int, default=None)
parser.add_argument('--output', help='path for dataset output', type=str, default=None)
parser.add_argument('--map', help='map for data collocte', type=str, default=None)
parser.add_argument('--num_vehicles', help='the number of npc vehicle', type=int, default=None)
parser.add_argument('--frame', help='the number of frame', type=int, default=None)
args = parser.parse_args()

cfg = make_cfg(args)
# print(cfg)
