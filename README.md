# Romea Core Control

The **Romea Core Control** library provides a robust set of control algorithms designed to enhance robotic coordination and precision. It includes features such as **Formation Control**, which enables the synchronized movement of multiple robots, and **Trajectory Tracking** for the accurate following of predefined paths. Additionally, the library offers **Slip Estimation** algorithms to detect and compensate for slip during robot motion.


## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-control/refs/heads/main/romea_control_public.repos
5. vcs import src < romea_romea_control_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to Project Title, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

Project Title is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Control library, created by **Jean Laneurit**, is the result of extensive research conducted by the robotic team of the **INRAE  TSCF** unit. This project benefited from the valuable contributions of researchers, PhD students, postdoctoral fellows, and external partners, all working under the supervision of **Roland Lenain**.

TODO add list of contributors

## **Contact**

If you have any questions or comments about Romea Core Control, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** or **[Roland Lenain](mailto:roland.lenain@inrae.fr)** 