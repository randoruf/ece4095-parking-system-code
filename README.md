# Autonomous Parking for Self-Driving Cars

The aim of this project is to deal with parking spot assignments in a smart parking lot. 

There are a lot of academic publications focus on a single vehicle, but the parking problem involved with multi-agent has not been extensively studied yet. The ultimate goal is to develop a robust closed-loop goal assignment algorithm that is specified for automated valet parking scenarios. For now, the project is only at the very beginning phase… Hopefully, I will have time to solve the dynamic scenarios (e.g. vehicles can come in from the entrance or get out of the scene via the exit) in the future.

The project assumes the parking lot can percept the environment by using the sensors and automatically assign parking spots to incoming cars.

![prioritized_goal_example_presentation](./imgs/prioritized_goal_example_presentation.gif)


## Acknowledgement

Thanks for Akansel Cosgun providing this interesting idea!

## Set Up 

- Install Visual Studio 2017 <https://visualstudio.microsoft.com/vs/older-downloads/>
- Set up the configurations for OMPL, SDL2 by using [vcpkg](https://github.com/microsoft/vcpkg)
  - SDL2 is deprecated because I was thinking for online simulation previously, but it turns out offline simulation is enough... 
  - OMPL http://ompl.kavrakilab.org/installation.html
  - SDL2 and Visual Studio [Setup SDL2 with VS2019 and vcpkg](https://randoruf.github.io/2021/03/23/install-sdl2-windows.html)
- Set up Python3 environment https://www.anaconda.com/products/individual
- Open Project in Visual Studio 2017

