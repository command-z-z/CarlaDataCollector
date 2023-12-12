from lib.collectors import make_collector
from lib.clients import make_client
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
        for _ in tqdm(range(cfg.all_frame_iter)):
            data = client.tick()
            collector.save_training_files(data)

    finally:
        client.setting_recover()


if __name__ == '__main__':
    main()
