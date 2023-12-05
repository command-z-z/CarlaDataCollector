import imp

def make_client(cfg):
    module = cfg.client_module
    path = cfg.client_path
    client = imp.load_source(module, path).Client(cfg)
    return client

