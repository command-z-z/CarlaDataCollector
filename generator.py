from lib.datasets import DataSave
from lib.mode import SynchronyMode
from lib.utils.data_utils import objects_filter
from lib.config import cfg
from tqdm import tqdm

def main():
    model = SynchronyMode(cfg)
    dtsave = DataSave(cfg)
    try:
        model.set_synchrony()
        model.spawn_actors()
        model.set_actors_route()
        model.spawn_agent()
        model.sensor_listen()
        step = 0
        STEP = cfg["save_config"]["step"]
        NUM_STEP = cfg["save_config"]["num_step"]
        for _ in tqdm(range(NUM_STEP)):
            if step % STEP == 0:
                data = model.tick()
                data = objects_filter(data)
                dtsave.save_training_files(data)
            else:
                model.world.tick()
            step+=1

    finally:
        model.setting_recover()


if __name__ == '__main__':
    main()
