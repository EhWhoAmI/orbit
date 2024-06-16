Orbit math
==========
This is an extraction of the [Conquer Space](https://github.com/Conquer-Space/Conquer-Space) orbit math.

This is a [patched conic](https://en.wikipedia.org/wiki/Patched_conic_approximation) orbital model. This uses sources from various papers and books, some of them are documented within the code, while *Fundamentals of Astrodynamics and Applications* by Vallado was also extremely useful to get to this point.

The critical files to take note of are `orbit.h` and `orbit.cpp`, that is where all the equations are.

`movement.h` and `movement.cpp` is a rough guide on how to run the guide but I cannot assure that any of the code in those two files will compile at all.

Generically, a body will need a `Orbit` and a `Kinematics` component to be a functioning body.

You can place a planet and attach a `OrbitalSystem` to it, and append all the objects orbiting that planet (including other planet and moons) to the vector contained within the `OrbitalSystem`.

Then you can run `SysOrbit` and it will help process everything for you at the interval that you want.
