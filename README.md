# Focal Path Guiding for Light Transport Simulation

![Teaser](/assets/teaser.jpg)

This repository contains the authors' Mitsuba implementation of the 
[Focal Path Guiding algorithm](https://graphics.cg.uni-saarland.de/publications/rath-sig2023.html).
We have implemented our algorithm in a recursive path tracer, which can be found under [`mitsuba/src/integrators/path/focal_path.cpp`](mitsuba/src/integrators/path/focal_path.cpp).
The underlying data structure, as described in our paper, can be found under [`mitsuba/src/integrators/path/focal_guiding.h`](mitsuba/src/integrators/path/focal_guiding.h).

## Parameters

⚠️ Please note that this implementation does not support Russian roulette. Common variants of Russian roulette are prone to undoing the benefits of guiding. Given its orthogonality, we leave implementing sophisticated variants as future work.

Our algorithm performs multiple training iterations before producing the final render. For more details, please refer to our paper.
The following parameters are supported by our integrator:

### `budget` (default 120)
The time (in seconds) allocated for rendering.

### `iterationCount` (default 15)
The number of training iterations to be performed.

### `iterationBudget` (default 6)
The time (in seconds) that each training iteration takes.
A good initial guess is to dedicate half of the time to training and the other half to rendering [Müller et al. 2017].
To this end, make sure that iterationCount times iterationBudget equals half of the budget.

### `dumpScene` (default false)
Dumps the geometry of the scene as single PLY mesh file, used by our visualizer.

### `orth.threshold` (default 1e-3)
Controls the resolution of the guiding octrees. Smaller values will yield finer resolution,
at the expense of higher computational cost per sample.
If many focal points are present, consider lowering this value.
Please consult our paper for a study on this parameter.

## Compilation

To compile the Mitsuba code, please follow the instructions from the [Mitsuba documentation](http://mitsuba-renderer.org/docs.html) (sections 4.1.1 through 4.6). Since our new code uses C++11 features, a slightly more recent compiler and dependencies than reported in the mitsuba documentation may be required. We only support compiling mitsuba with the [scons](https://www.scons.org) build system, but we do support Python 3.

We tested our Mitsuba code on
- macOS (Ventura, `arm64`)
- Linux (Ubuntu 22.04, `x64`)

## License

The new code introduced by this project is licensed under the GNU General Public License (Version 3). Please consult the bundled LICENSE file for the full license text.
