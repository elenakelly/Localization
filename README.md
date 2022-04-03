# Localization
* Main task: "Self-localization of mobile robot with Kalman Filter"</br>


CODE DIVISION : </br>

Elena Kane i6289291 - motion move, filter, visuals in time steps </br>
Nikolaos Ntantis i6273751 - localization, noise to movement, noise to sensor  </br>
Ioannis Montesantos i6292068 - sensors, beacons, calculation of landmark's distances</br>


Visuals Representation:</br>
Ellipse =  shows intermediate estimates of position and covariance</br>
Dotted line = estimated robot trajectory</br>
Solid line = actual robot trajectory</br>
Black point = beacons/landmarks/features</br>
Green line = sensor range between robot and beacon</br>


Discription:</br>
• Velocity-based motion model (𝑢 = (𝑣, 𝜔)𝑇)</br>
• Control robot with key board (W=increment 𝑣, S=decrement 𝑣, A=decrement 𝜔 D=increment 𝜔, X=stop)</br>
• No walls, no collision with features</br>
• Point-based features </br>
• Omnidirectional sensor for feature detection</br>
• Limited sensor range</br>
• Bearing and distance estimate</br>
• Known correspondence</br>
• Track pose with Kalman Filter</br>

## Installation
The program is in Python <br />
In order to use the code you need to install Numpy, Math and Pygame packages
   ```sh
    import numpy as np
    import pygame
    import math
   ```
 You can start by running the
 ### `Robot`

## Contributors
Elena Kane </br>
Nikolaos Ntantis </br>
Ioannis Montesantos 

