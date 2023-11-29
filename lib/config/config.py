import yaml
import carla
import argparse


def cfg_from_yaml_file(cfg_file):
    with open(cfg_file, 'r') as f:
        try:
            config = yaml.load(f, Loader=yaml.FullLoader)
        except:
            config = yaml.load(f)

    return config

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
parser.add_argument('--test', action='store_true', dest='test', default=False)
args = parser.parse_args()

cfg = cfg_from_yaml_file(args.cfg_file)
