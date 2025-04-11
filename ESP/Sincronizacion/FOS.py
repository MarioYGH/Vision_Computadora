import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit


def fopdt(t, Kp, tau, theta):
    """
     Kp: the gain of the process
     tau: time constant of the process
     theta: dead time of the system
    """

    y = np.zeros_like(t)
    t_ = np.zeros_like(t)

    # Calculate the difference in u values
    delta_u = np.diff(u, prepend=0)

    n = np.where(delta_u != 0)[0][0]

    t_[n:] = t[:-n]

    mask = t_ > theta

    y[mask] = Kp * u[mask] * (1 - np.exp(-(t_[mask] - theta) / tau))

    return y


if __name__ == '__main__':
    with open('input_response.npy', 'rb') as f:
        u = np.load(f)
        y = np.load(f)
        t = np.load(f)
        h = np.load(f)

    y = np.nan_to_num(y)

    # Initial parameter guesses: Kp, tau, theta
    initial_guesses = [np.random.rand(), np.random.rand(), np.random.rand()]

    # Curve fitting
    popt, _ = curve_fit(fopdt, t, y, p0=initial_guesses)
    Kp, tau, theta = popt

    y_hat = fopdt(t, Kp, tau, theta)

    print(f"Kp = {np.round(Kp, 4)}")
    print(f"tau (time constant)= {np.round(tau, 4)}")
    print(f"theta (dead time) = {np.round(theta, 4)}")

    plt.figure()
    plt.plot(t, y_hat, '--', label='$\hat{y}(t)$')
    plt.plot(t, y, label='y(t)')
    plt.plot(t, u, label='u(t)')
    plt.legend(loc='upper left')

    plt.show()
