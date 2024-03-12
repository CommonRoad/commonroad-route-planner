import os
import shutil


# typing
from typing import List


def main():
    """
    Expected Folder structure:

    root
    |- scenario folder
        |- scenario name
            |- scenario files

    """
    extract_root: str = "/home/tmasc/Downloads/CRMPC2023_Scenarios"
    save_path: str = "/home/tmasc/route_planner/commonroad-route-planner/tutorial/commonroad_challenge_2023"

    scenario_dirs: List[str] = os.listdir(extract_root)

    for scenario_dir in scenario_dirs:
        path = os.path.join(extract_root, scenario_dir)
        scenario_names: List[str] = os.listdir(path)

        for scenario in scenario_names:
            path = os.path.join(extract_root, scenario_dir, scenario)
            scenario_files: List[str] = os.listdir(path)

            for scenario_file in scenario_files:
                if ".cr." in scenario_file:
                    file_path: str = os.path.join(path, scenario_file)
                    new_name: str = scenario_file.split(".")[0] + ".xml"
                    new_file_path: str = os.path.join(save_path, new_name)

                    shutil.copy(file_path, new_file_path)


if __name__ == "__main__":
    main()
