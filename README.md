# Self-localization of mobile robot with Kalman Filter

 Simulator programmed in Python with PyGame.
 You can start by running the file:
 ### `Robot`
It  has localization implemented using landmarks with known correspondence and using Kalman Filter to correct the measurement noise.

## Discription:
• Velocity-based motion model (𝑢 = (𝑣, 𝜔)𝑇)</br>
• Control robot with key board (W=increment 𝑣, S=decrement 𝑣, A=decrement 𝜔 D=increment 𝜔, X=stop)</br>
• No walls, no collision with features</br>
• Point-based features </br>
• Omnidirectional sensor for feature detection</br>
• Limited sensor range</br>
• Bearing and distance estimate</br>
• Known correspondence</br>
• Track pose with Kalman Filter</br>

## Visuals Representation:</br>
*Ellipse =  shows intermediate estimates of position and covariance</br>
*Dotted line = estimated robot trajectory</br>
*Solid line = actual robot trajectory</br>
*Black point = beacons/landmarks/features</br>
*Green line = sensor range between robot and beacon</br>

## Installation
The program is in Python <br />
In order to use the code you need to install Numpy, Math and Pygame packages
   ```sh
    import numpy as np
    import pygame
    import math
   ```

## Contributors
Elena Kane </br>
Nikolaos Ntantis </br>
Ioannis Montesantos 

