import imp
from turtle import position
import numpy as np

from odp.Grid import Grid
from odp.Shapes import ShapeRectangle

from odp.dynamics import DubinsCar

from odp.Plots import PlotOptions
from odp.Plots import visualize_plots

from odp.solver import HJSolver, computeSpatDerivArray

from odp.compute_trajectory import find_sign_change, compute_opt_traj, spa_deriv

import math
import os

if os.path.exists("plots") == False:
    os.mkdir("plots")
    
# Step 1: Define grid
grid_min = np.array([-1.0, -1.0, -np.pi])
grid_max = np.array([1.0, 1.0, np.pi])
dims = 3
N = np.array([40, 40, 40])
pd = [2]
g = Grid(grid_min, grid_max, dims, N, pd)

# Step 2: Generate initial values for grid using shape functions
goal_min = [0.6, 0.1, -np.pi]
goal_max = [0.8, 0.3, np.pi]
Initial_value_f = ShapeRectangle(g, goal_min, goal_max)

# TODO: Figure out how to include obstacles.
# obstacle = ShapeRectangle(
#         g,
#         [-0.1, 0.3, -math.pi], 
#         [0.1, 0.6, math.pi]
# )

# obstacle = ShapeRectangle(
#         g,
#         [-0.1, -1.0, -math.pi], 
#         [0.1, -0.3, math.pi]
# )

# Step 3: Time length for computations
Lookback_length = 1.0
t_step = 0.1

small_number = 1e-5
tau = np.arange(start=0, stop=Lookback_length + small_number, step=t_step)

# Step 4: System dynamics for computation
# uMode set to min for reaching the target, trying to minimize the value function
car = DubinsCar(uMode="min", dMode="max")  # Define system

# Step 5: Call HJSolver function
compMethod = {"TargetSetMode": "minVOverTime"}
result = HJSolver(car, g, Initial_value_f, tau, compMethod, saveAllTimeSteps=True)

# Visualization of 3D value function
po = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2], slicesCut=[50],colorscale="Bluered", 
                 save_fig=True, filename="plots/3D_0_sublevel_set", interactive_html=True)
visualize_plots(result, g, po)

# Step 6: Compute spatial derivatives of the value function

position = np.array([0.0, 0.0, 0.0])  # Initial position of the agent

def goal_reached(position):
        return (goal_min[0] <= position[0] <= goal_max[0]) and (goal_min[1] <= position[1] <= goal_max[1]) 
        

optimal_controls = []
while not goal_reached(position):

        neg2pos, pos2neg = find_sign_change(g, result, position, tau)
        assert result.shape[-1] == len(tau)  # check the shape of value function

        # check the current state is in the reach-avoid set
        current_value = g.get_values(result[..., 0], list(position))
        if current_value > 0:
                result = result - current_value

        # calculate the derivatives
        v = result[..., neg2pos] # Minh: v = result[..., neg2pos[0]]
        # print(f"The shape of the input value function v of attacker is {v.shape}. \n")
        spat_deriv_vector = spa_deriv(g.get_indices(position), v, g)      

        optimal_controls.append(car.optCtrl_inPython(spat_deriv_vector))

        # update the position of the agent using the dynamics
        position = car.forward(ctrl_freq=1/t_step, current_state=position, u=optimal_controls[-1])


# Step 7: Export optimal control values to a file
with open('dubins_control.txt', "w") as f:
        for control in optimal_controls:
                f.write(f"{float(control)}\n")

