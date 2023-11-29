from lib.collectors import make_collector
from lib.clients import make_client
from lib.utils.data_utils import objects_filter
from lib.config import cfg
from tqdm import tqdm

def main():
    client = make_client(cfg)
    collector = make_collector(cfg)

    try:
        client.set_synchrony()
        client.spawn_npc()
        client.set_npc_route()
        client.spawn_ego_vehicle()
        client.sensor_listen()
        for frame in tqdm(range(cfg.save_frame_iter * cfg.all_frame_iter)):
            if frame % cfg.save_frame_iter == 0:
                data = client.tick()
                data = objects_filter(data)
                collector.save_training_files(data)
            else:
                client.world.tick()

    finally:
        client.setting_recover()


if __name__ == '__main__':
    main()
