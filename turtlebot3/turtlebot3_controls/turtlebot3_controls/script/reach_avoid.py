import imp
import numpy as np

from odp.Grid import Grid
from odp.Shapes import ShapeRectangle

from odp.dynamics import DubinsCar

from odp.Plots import PlotOptions
from odp.Plots import visualize_plots

from odp.solver import HJSolver, computeSpatDerivArray

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

# Step 3: Time length for computations
Lookback_length = 1.0
t_step = 0.1

small_number = 1e-5
tau = np.arange(start=0, stop=Lookback_length + small_number, step=t_step)

# Step 4: System dynamics for computation
# uMode set to min for reaching the target, trying to minimize the value function
sys = DubinsCar(uMode="min", dMode="max")  # Define system

# Step 5: Call HJSolver function
compMethod = {"TargetSetMode": "minVOverTime"}
result = HJSolver(sys, g, Initial_value_f, tau, compMethod, saveAllTimeSteps=True)

# Visualization of 3D value function
po = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2], slicesCut=[50],colorscale="Bluered", 
                 save_fig=True, filename="plots/3D_0_sublevel_set", interactive_html=True)
visualize_plots(result, g, po)