from lib.utils.export_utils import *
from loguru import logger
import os
import shutil

class BasicDataCollector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.OUTPUT_FOLDER = self.cfg.result_dir
        self.captured_frame_no = self._current_captured_frame_num()

    def _current_captured_frame_num(self):
        image_path = os.path.join(self.OUTPUT_FOLDER, 'image/')
        if not os.path.exists(image_path):
            return 0
        num_existing_data_files = len(
            [name for name in os.listdir(image_path) if name.endswith('.png')])
        if num_existing_data_files == 0:
            return 0
        answer = input(
            "There already exists a dataset in {}. Would you like to (D)elete or (A)ppend the dataset? (D/A)".format(
                self.OUTPUT_FOLDER))
        if answer.upper() == "D":
            logger.info( 
                "Resetting frame number to 0 and Delete existing")
            self.delete_all_files_and_folders(self.OUTPUT_FOLDER)
            return 0
        logger.info("Continuing recording data on frame number {}", num_existing_data_files)
        return num_existing_data_files

    def delete_all_files_and_folders(self, folder_path):
        for item in os.listdir(folder_path):
            item_path = os.path.join(folder_path, item)
            if os.path.isdir(item_path):
                shutil.rmtree(item_path)
            else:
                os.remove(item_path)
