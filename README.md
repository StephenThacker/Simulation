# Simulation
Simulation w. Tensorflow Repository

Currently a work in progress. The concept of the project is to build a physics based simulation of an object being launched from a sling and then train a machine learning model to hit virtual targets, using reiniforcement learning. 

We assume that there are no affects of sling tension acting on the projectile and that the sling rotates through the z dimension of 3 dimensional Euclidean Space. The current version gives the first iteration of the physics based simulation for this projectile. While in the sling, a constant tangential force acts on the projectile and the projectile accelerates via the kinematic equations for centripetal motion. The projectile is released from the sling at a predetermined time and is affected by the force of gravity, until it hits the ground. In the next iteration of the project, the time of release will be controlled by a machine learning model to hit user selected targets. The images below shows the position and velocities of the object as the simulation progresses. The first image shows the objects path before leaving the sling and the second image shows the object's path after it leaves the sling. Please note that the two images are not on the same scale.  ![Figure1_prelaunch](https://github.com/StephenThacker/Simulation/assets/35053174/97e97a53-4e6a-43c6-b76a-7630747c7980) 

Fig 1: Position and Velocities of Projectile Before Leaving Sling

![post_launch](https://github.com/StephenThacker/Simulation/assets/35053174/a3cb1ed3-5259-434d-945a-0a0bb81dfe39) 

Fig 2: Positions and Velocities of Projectile After Leaving Sling
