# Simulation
Simulation w. Tensorflow Repository

Currently a work in progress. The concept of the project is to build a physics based simulation of an object being launched from a sling and then train a machine learning model to hit virtual targets, using reiniforcement learning. 

We assume that there are no affects of sling tension acting on the projectile and that the sling rotates through the z dimension. The current version gives the first version of the physics based simulation for this projectile via the basic kinematic equations. The images below shows the position and velocities of the object as the simulation progresses. The first image shows the objects path before leaving the sling and the second image shows the object's path after it leaves the sling. While in the sling, a constant tangential force acts on the object, later this will be controlled by the machine learning model. Please note that the two images are not on the same scale.  ![Figure1_prelaunch](https://github.com/StephenThacker/Simulation/assets/35053174/97e97a53-4e6a-43c6-b76a-7630747c7980)
![post_launch](https://github.com/StephenThacker/Simulation/assets/35053174/486a95ec-2c00-4a2a-902d-98254a9f3cbe)
