import numpy as np
from numpy import asarray
from numpy import exp
from numpy.random import randn
from numpy.random import rand
from numpy.random import seed


def simulated_annealing(objective, x0, upper_bounds, lower_bounds, constrain, n_iterations=1000, step_size=0.1, temp=10, args=()):
    # Modified from: https://machinelearningmastery.com/simulated-annealing-from-scratch-in-python/

    best = x0
    best_eval = objective(best, *args)

    # current working solution
    curr, curr_eval = best, best_eval

    for i in range(n_iterations):
        # take a step
        candidate = curr + randn(len(upper_bounds)) * step_size
        print("generated candidates")

        # Clamp the guess within the bounds
        candidate = np.clip(candidate, lower_bounds, upper_bounds).tolist()
        print("clamped candidates in bounds")

        # Dynamically constrain the candidate
        candidate = constrain(candidate, best)
        print("constrain candidates")

        # evaluate candidate point
        candidate_eval = objective(candidate, *args)
        print("evaluated guess")

        # check for new best solution
        if candidate_eval < best_eval:
            # store new best point
            best, best_eval = candidate, candidate_eval
            # report progress
            # print(">%d f(%s) = %.5f" % (i, best, best_eval))

        # difference between candidate and current point evaluation
        diff = candidate_eval - curr_eval

        # calculate temperature for current epoch
        t = temp / float(i + 1)

        # calculate metropolis acceptance criterion
        metropolis = exp(-diff / t)

        # check if we should keep the new point
        if diff < 0 or rand() < metropolis:
            # store the new current point
            curr, curr_eval = candidate, candidate_eval

    return [best, best_eval]
