# import cma
# import numpy as np
#
#
# # def runOptimization(robo_task, max_iter=20, solver_type="RoBO", cost_saturation=2.0):
#     optimal_params = []
#     optimal_cost = []
#     original_cost = []
#
#     optimization_converged = False
#
#
#     X1 = robo_task.X_init_original
#     Y1 = robo_task.Y_init_original
#     X_lowerBound = robo_task.X_lower_original
#     X_upperBound = robo_task.X_upper_original
#
#     X_range = X_upperBound - X_lowerBound
#     X1_norm = (X1 - X_lowerBound)/X_range
#     cmaes_solver = cma.CMAEvolutionStrategy((X), 0.5)
#
#     while not cmaes_solver.stop() and (robo_task.optimization_iteration <= max_iter):
#         solutions = cmaes_solver.ask()
#         costs = []
#         for s in solutions:
#             cost = robo_task.objective_function(np.array(s))
#             cost_norm = cost/Y1
#             cost_norm = np.where((y_norm>cost_saturation), cost_saturation, y_norm)
#             costs.append(cost_norm)
#
#         cmaes_solver.tell(solutions, costs)
#         cmaes_solver.logger.add()  # write data to disc to be plotted
#         cmaes_solver.disp()
#
#     cmaes_solver.result_pretty()
#     cma.plot()  # shortcut for cmaes_solver.logger.plot()
    # plt.show(block=True)
