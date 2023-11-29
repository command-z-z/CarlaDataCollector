import os
import imp

def make_collector(cfg):
    module = cfg.collector_module
    path = cfg.collector_path
    collector = imp.load_source(module, path).DataCollector(cfg)
    return collector

