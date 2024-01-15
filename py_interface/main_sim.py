import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from controller.PID import PID

def motor_model(y, t, J, b, Kt, L, R, Ke):
    dth_dt, i = y
    X = np.array([dth_dt, i])
    A = np.array([[-b/J, Kt/J], [-Ke/L, -R/L]])
    B = np.array([0, 1.0/L])

    dth_dt_0 = 100
    K = 10
    u = K * (dth_dt_0 - dth_dt)
    V = np.array([u, u])
    dX_dt = np.matmul(A, X) + np.multiply(B, V)

    return dX_dt

def main():
    J = 0.01
    b = 0.1
    Kt = 0.01
    Ke = 0.01
    R = 1.0
    L = 0.1

    initial_conditions = [0.0, 0.0]
    t = np.linspace(0, 10, 2500)
    solution = odeint(motor_model, initial_conditions, t, args=(J, b, Kt, L, R, Ke))

    angular_velocity, current = solution[:, 0], solution[:, 1]

    plt.plot(t, angular_velocity, label='Angular Velocity')
    # plt.plot(t, current, label='Current')
    plt.xlabel('Time')
    plt.ylabel('Motor Response')
    plt.title('DC Motor Model with Voltage Input')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
