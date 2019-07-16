# ros2_qbot_1

Python and C++ robotics code, most of which is compliant with ROS 2.0 Dashing, with a few non-ROS2 Python scripts for general purpose testing.  The code in this repo is intended to interact with the code in repo "freertos_stm32_motor_control", to form a "complete" robot control firmware/software implementation.  The ROS2 code runs on Ubuntu 18.04 on an embedded ARM board, currently an Odroid-N2, whereas the FreeRTOS code in the other repo currently runs on a STM32F401 Nucleo64 board.  The src folder contains some ROS and some non-ROS code.  Portions of the code acts as a communication and supervisory interface to facilitate communication with the freertos code.

Folder structure for Python ROS2 nodes:

workspace
    |_src (folder)
        |_repo_name (folder)
            |_CONTRIBUTING.md
            |_LICENSE
            |_non_ros2_file_folder
                |_non_ros2_file_1.py
                |_non_ros2_file_2.py
                |_etc...
            |_package_name (folder)
                |_setup.cfg
                |_setup.py
                |_package.xml
                |_resource (folder)
                    |_package_name (empty file, no extension)
                |_test (folder)
                    |_test_copyright.py
                    |_test_flake8.py
                    |_test_pep257.py
                |_node_source_folder
                    |___init__.py (empty file)
                    |_source_file_1.py
                    |_source_file_2.py
                    |_etc...
