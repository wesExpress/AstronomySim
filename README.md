# AstronomySim

Prototype for an astronomy simulation. This project uses my [c99 framework DarkMatter](https://github.com/wesExpress/DarkMatter) for rendering, physics, and general ECS.

Main goals:
  - Accurate N-body simulation
  - Ability to land/move on said N-bodies
  - Ability to 'observe' said N-bodies in real time
    - Take images (generate textures?)
    - Make star/galaxy catalogues
    - Use measurements to attempt to travel to other planets/stars
    
For this to work, I will need to scale the 'universe' down considerably for two reasons:
  - In order to be able to observe/see anything, objects must be much closer than in reality. Our eyes do not work as pixels do, so extremely distant objects will not render correctly
  - Travel distance must be reduced in order to have anything reasonable
  
  
### Long-shot goals

- Hydrodynamic simulation: gas and dust clouds are a huge part of astronomy
- As realistic lighting as possible
  - Ray tracing would be ideal, but let's be honest that's not happening
