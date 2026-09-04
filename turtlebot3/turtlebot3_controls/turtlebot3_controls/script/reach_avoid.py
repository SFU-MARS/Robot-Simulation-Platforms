import imp
import numpy as np
# Utility functions to initialize the problem
from odp.Grid import Grid
from odp.Shapes import ShapeRectangle

# # Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCar2
# Plot options
from odp.Plots import PlotOptions, visualize_plots

# Solver core
from odp.solver import HJSolver, computeSpatDerivArray
import math

from odp.Grid import Grid

from odp.Plots import plot_isosurface, plot_valuefunction

from odp.solver import HJSolver
from odp.Plots.plot_options import PlotOptions

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator


# Define state space grid [x, y, theta]
grid_min = np.array([0.0, 0.0, -math.pi])
grid_max = np.array([1.0, 1.0,  math.pi])
dims = 3
N = np.array([101,101,51])
pd = [2]      # heading is periodic
g = Grid(grid_min, grid_max, dims, N, pd)


# Define goal and obstacle sets
goal = ShapeRectangle(
        g,
        [0.6,  0.1, math.pi],
        [0.8,0.3, -math.pi],
)
obstacle = ShapeRectangle(
        g,
        [-0.1, 0.3, -math.pi], 
        [0.1, 0.6, math.pi]
)

obstacle = ShapeRectangle(
        g,
        [-0.1, -1.0, -math.pi], 
        [0.1, -0.3, math.pi]
)


# Define dynamics and solver parameters
# 3d two controls
# 4d for maybe 
robot = DubinsCar2(
        x=[0.0, 0.0, 0.0],
        uMin=np.array([0.2, -0.5]),
        uMax=np.array([0.8, 0.5])
)
        
# change the initial states if the robot goes out of bounds
# value function only computed within the grid no walls
# constrain the dynamics to be a 2d dubins car

lookback = 5.0
dt = 0.05
tau = np.arange(0,
                lookback+1e-5,
                dt)


compMethod = {
    "TargetSetMode":"None"
}

result = HJSolver(
                robot,
                g,
                goal,
                tau,
                compMethod,
                # save all time steps for the reach problem
                saveAllTimeSteps=True
        )
       

# NOTE: avoid depends on robot dynamics

# Visualize the results
po = PlotOptions(
        do_plot=True,
        plot_type="set",
        plotDims=[0,1,2])

plot_isosurface(g,result,po)
theta_index = 25

costmap = result[:,:,theta_index]

Vx = np.gradient(costmap, axis=0)
Vy = np.gradient(costmap, axis=1)

Vtheta = np.gradient(
    result,
    g.dx[2],
    axis=2
)

# Compute optimal control values

# ---------------------------------------
# Interpolate Vtheta
# ---------------------------------------

x_grid = g.grid_points[0]
y_grid = g.grid_points[1]
theta_grid = g.grid_points[2]

Vtheta_interp = RegularGridInterpolator(
    (x_grid, y_grid, theta_grid),
    Vtheta,
    bounds_error=False,
    fill_value=None
)

# Output optimal control values


# TODO: Store all time slices of the value fucntion for the reach problem into a file