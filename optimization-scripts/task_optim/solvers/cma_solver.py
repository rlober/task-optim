from .base_solver import BaseSolver
import cma
import numpy as np

class CmaSolver(BaseSolver):
    """docstring for CmaSolver."""
    def __init__(self, test, solver_parameters):
        assert('max_iter' in solver_parameters)
        self.initial_sigma = 0.5
        self.tolfun = 1e-11

        super(CmaSolver, self).__init__(test, solver_parameters)

    def testSolution(self, s):
        next_solution_to_test = np.array(s)
        # The provided solution is scaled bectween 0-1 so we retransform it back to physical coordinates
        X_new = self.test.retransform(next_solution_to_test)
        # test X_new
        Y_new = self.test.objective_function(X_new)
        # scale the cost wrt the original cost
        Y_new = ( Y_new / self.test.Y_init )

        self.X = np.vstack((self.X, X_new))
        self.Y = np.vstack((self.Y, Y_new))

        return Y_new.flatten()[0]

    def initializeSolver(self):
        if 'initial_sigma' in self.solver_parameters:
            if self.solver_parameters['initial_sigma'] > 0.0:
                self.initial_sigma = self.solver_parameters['initial_sigma']

        if 'tolfun' in self.solver_parameters:
            if self.solver_parameters['tolfun'] > 0.0:
                self.tolfun = self.solver_parameters['tolfun']

        # Construct CMAES solver
        X = self.test.transform(self.X)
        self.cmaes_solver = cma.CMAEvolutionStrategy(X.flatten().tolist(), self.initial_sigma, {'bounds': [0, 1], 'tolfun':self.tolfun})

    def updateSolver(self):
        solutions = self.cmaes_solver.ask()
        costs = []
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("Testing new solution batch")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        for i, s in enumerate(solutions):
            print("\n--> Testing solution " + str(i) + " of "+str(len(solutions))+"\n")
            costs.append(self.testSolution(s))

        self.cmaes_solver.tell(solutions, costs)
        # self.cmaes_solver.logger.add()
        self.cmaes_solver.disp()


    def solverFinished(self):
        if not self.cmaes_solver.stop() and (self.test.optimization_iteration <= self.solver_parameters['max_iter']):
            return False

        else:
            if self.test.optimization_iteration > self.solver_parameters['max_iter']:
                print("Maximum number of iterations exceeded. Stopping optimization.")
            else:
                print("CMA-ES Solver has converged. Stopping optimization.")

            print("\nOptimization complete - testing best incumbent solution...")
            best_x = self.cmaes_solver.result()[0]
            self.testSolution(best_x)
            print("Finished.")

            return True
