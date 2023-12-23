from lib.utils.export_utils import *
from loguru import logger
import os
import shutil

class BasicDataCollector:
    """
    BasicDataCollector is a class responsible for collecting and managing data in a simulation environment.

    This class handles the creation and management of output folders for simulation data, keeps track of the number of frames captured,
    and provides functionality to reset or append to existing datasets.

    Attributes:
        cfg: Configuration object containing settings for data collection and storage.
        OUTPUT_FOLDER (str): Directory path where the simulation results are stored.
        captured_frame_no (int): The current frame number, indicating how many frames have been captured.

    Methods:
        _current_captured_frame_num: Determines the current frame number based on existing data in the output folder.
        delete_all_files_and_folders: Deletes all files and folders within a specified directory.
    """

    def __init__(self, cfg):
        """Initializes the BasicDataCollector class.

        Sets up the output folder for storing simulation results and determines the current frame number based on existing data.

        Args:
            cfg: Configuration object containing settings for data collection and storage.

        """
        self.cfg = cfg
        self.OUTPUT_FOLDER = self.cfg.result_dir
        self.captured_frame_no = self._current_captured_frame_num()

    def _current_captured_frame_num(self):
        """Determines the current frame number based on existing data in the output folder.

        Checks for existing image files in the output folder to decide whether to start from frame 0 or append to existing data.
        Offers the user a choice to delete or append to the existing dataset.

        Returns:
            int: The frame number from which data collection will commence.

        """
        image_path = os.path.join(self.OUTPUT_FOLDER, 'rgb/')
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
        """Deletes all files and folders within the specified directory.

        This method is used to clear the existing dataset in the output folder if the user chooses to do so.

        Args:
            folder_path (str): The path of the folder to be cleared.

        """
        for item in os.listdir(folder_path):
            item_path = os.path.join(folder_path, item)
            if os.path.isdir(item_path):
                shutil.rmtree(item_path)
            else:
                os.remove(item_path)
