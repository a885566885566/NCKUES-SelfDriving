import numpy as np
import Simulation

def main():
    import matplotlib.pyplot as plt
    import WaypointsPlanner as wp
    import VelocityProfile 
    vel_config = VelocityProfile.VelocityConfig(10, 0.1, 0.1, -0.5)
    way = wp.WaypointsPlanner(vel_config)
    sim = Simulation.Simulation(way, 0.1)
    a = 0.01
    b = 0.01

    state = np.zeros((10,4))
    state[:,0] = np.arange(10)
    state[:,1] = a*state[:,0]**3 + b*state[:,0]**2

    state[:,2] = np.tan(3*a*state[:,0]**2 + 2*b*state[:,0])
    state[:,3] = 10*np.ones(10)

    current_state = state[0]

    for s in state:
        delta = np.arctan(0.5*np.tan(s[2]))
        sim.plot_vehicle(s)
    plt.show()

if __name__ == "__main__":
    main()
