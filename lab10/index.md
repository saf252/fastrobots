---
---

# Lab 10: Grid Localization using Bayes Filter

## Code

I implemented the function stubs using the equations and processes described in lecture, and the speedup tips given in the lab assignment.

<script src="https://gist.github.com/saf252/aba10d93ffe334937367f56cd99664dd.js"></script>

## Simulation

<iframe width="560" height="315" src="https://www.youtube.com/embed/P1ErnuIviJ0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe> This worked shockingly well considering how far off the odometry data was, and it still worked for even worse odometry in other runs.

## Analysis

| Iteration | Belief (index) | Probability | Ground Truth (index) |
| --------: | :------------: | :---------: | :------------------: |
|         0 |   (6, 4, 6)    |     1.0     |      (6, 3, 6)       |
|         1 |   (6, 2, 5)    |     1.0     |      (7, 2, 5)       |
|         2 |   (7, 2, 4)    |     1.0     |      (7, 2, 4)       |
|         3 |   (6, 1, 4)    |     1.0     |      (7, 0, 4)       |
|         4 |   (8, 1, 9)    |     1.0     |      (8, 0, 9)       |
|         5 |  (10, 1, 11)   |     1.0     |     (11, 0, 11)      |
|         6 |  (10, 2, 12)   |     1.0     |     (11, 2, 12)      |
|         7 |  (11, 3, 13)   |     1.0     |     (11, 3, 13)      |
|         8 |  (11, 5, 14)   |  0.9994357  |     (11, 5, 14)      |
|         9 |  (11, 7, 16)   |     1.0     |     (11, 6, 16)      |
|        10 |   (9, 8, 17)   |  0.9997392  |     (10, 7, 17)      |
|        11 |   (7, 7, 3)    |  0.9999999  |      (7, 6, 4)       |
|        12 |   (6, 4, 6)    |     1.0     |      (6, 4, 6)       |
|        13 |   (5, 3, 2)    |     1.0     |      (6, 3, 2)       |
|        14 |   (4, 3, 1)    |     1.0     |      (4, 2, 1)       |
|        15 |   (3, 3, 0)    |  0.9994632  |      (3, 2, 0)       |

The belief is consistently very confidently (i.e. probability rounds to 1) within about one grid cell from the ground truth.
It is slightly less confident in positions with symmetry, where there are some nearby poses with similar true observations.
Better sensors could help solve this problem, as it would allow the filter to rely more on odometry than correction to the ToF data.
