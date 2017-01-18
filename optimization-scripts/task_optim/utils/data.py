import numpy as np


class TaskData(object):
    """docstring for TestData"""
    def __init__(self, time, expectedDuration, real, ref, waypoints, torques, comBounds, jacobians, jointPositions, jointLimits, name=""):
        self.data_truncated = False
        self.name = name
        self.time = time
        self.dt_vector = np.diff(self.time)
        self.dt = self.dt_vector.mean()
        self.expectedDuration = expectedDuration
        self.real = real
        self.ref = ref
        self.waypoints = waypoints
        self.torques = torques
        self.nTimeSteps = self.time.shape[0]
        self.duration = self.time[-1]
        self.comBounds = comBounds
        self.lower_bounds = self.comBounds[:,0]
        self.upper_bounds = self.comBounds[:,1]

        if self.name is "CoM":
            nRows = 3
        else:
            nRows = 6

        nCols = int(np.size(jacobians[0]) / nRows)
        # print('jacobians have', nRows, 'rows and', nCols, 'columns.')
        self.jacobians = []
        for j in jacobians:
            self.jacobians.append(j.reshape((nRows, nCols)))


        self.jointPositions = jointPositions
        self.lower_joint_limits = jointLimits[0,:]
        self.upper_joint_limits = jointLimits[1,:]

        self.setBetaFactor(1.0)
        self.setEnergyScalingFactor(1e-4)


        if self.ref.shape[0] != self.nTimeSteps:
            print("Ref shape doesn't match time's shape...")
        if self.real.shape[0] != self.nTimeSteps:
            print("Real shape doesn't match time's shape...")
        if self.torques.shape[0] != self.nTimeSteps:
            print("Torques shape doesn't match time's shape...")

    def cutOffDataAfter(self, secs):
        for i,t in enumerate(self.time):
            if t >= secs:
                cuttoff_index = i
                break
        print('Truncating data at time =', t, 'and index =', cuttoff_index)

        self.time = self.time[:cuttoff_index]
        self.real = self.real[:cuttoff_index :]
        self.ref = self.ref[:cuttoff_index, :]
        self.torques = self.torques[:cuttoff_index :]
        self.jointPositions = self.jointPositions[:cuttoff_index,:]
        self.jacobians = self.jacobians[:cuttoff_index]

        self.dt_vector = np.diff(self.time)
        self.dt = self.dt_vector.mean()
        self.nTimeSteps = self.time.shape[0]
        self.duration = self.time[-1]
        self.setBetaFactor(1.0)
        self.setEnergyScalingFactor(1e-4)
        self.data_truncated = True

    def setBetaFactor(self, newBeta):
        """Sets the exponential factor for goal cost scaling.

        :param newBeta: see :eq:`gammaFactor`
        """
        self.beta = newBeta
        self.gamma = self.calculateGammaFactors()

    def setEnergyScalingFactor(self, newFactor):
        self.energy_scaling_factor = newFactor

    def start(self):
        return self.waypoints[0, :]

    def goal(self):
        if self.data_truncated:
            return self.ref[-1, :]
        else:
            return self.waypoints[-1, :]

    def nMiddleWaypoints(self):
        return self.waypoints.shape[0]-2

    def middleWaypoints(self):
        return self.waypoints[1:-1, :]

    def middleWaypointsFlattened(self):
        return self.middleWaypoints().flatten()

    def nDof(self):
        return self.real.shape[1]

    def positionErrorNorm(self):
        """Calculate the :math:`l_{2}` norm of the position tracking error.

        .. math::
            :label: positionErrorNorm

            \\boldsymbol{\\epsilon}_{\\text{position}} = \| \\mathcal{P}^{ref} - \\mathcal{P}^{real} \|

        Here, :math:`\\mathcal{P}` is a matrix of positions at each time step :math:`i\\in [1,n]`, :math:`\\boldsymbol{p}_{i}=\\begin{bmatrix}x & y & z\\end{bmatrix}`, such that :math:`\\mathcal{P} = \\begin{bmatrix}\\boldsymbol{p}_{1} \\\\ \\boldsymbol{p}_{2} \\\\ \\vdots \\\\ \\boldsymbol{p}_{n} \\end{bmatrix}`.
        """
        return np.linalg.norm((self.ref - self.real), ord=2, axis=1)

    def positionErrorSquaredNorm(self):
        """Calculate the squared :math:`l_{2}` norm of the position tracking error.

        .. math::
            :label: positionErrorSquaredNorm

            \\boldsymbol{\\epsilon}^{2}_{\\text{position}} = \| \\mathcal{P}^{ref} - \\mathcal{P}^{real} \|^{2}

        See :func:`positionErrorNorm` for details on :math:`\\mathcal{P}`.
        """
        return self.positionErrorNorm()**2

    def goalPositionErrorNorm(self):
        """Calculate the :math:`l_{2}` norm of the goal tracking error.

        .. math::
            :label: goalPositionErrorNorm

            \\boldsymbol{\\epsilon}_{\\text{goal}} = \| \\boldsymbol{p}_{goal} - \\mathcal{P}^{real} \|

        Here, :math:`\\boldsymbol{p}_{goal}` is the final waypoint of the task (see :func:`goal`). See :func:`positionErrorNorm` for details on :math:`\\mathcal{P}`.
        """
        return np.linalg.norm((self.goal() - self.real), ord=2, axis=1)

    def goalPositionErrorSquaredNorm(self):
        """Calculate the squared :math:`l_{2}` norm of the goal tracking error.

        .. math::
            :label: goalPositionErrorSquaredNorm

            \\boldsymbol{\\epsilon}^{2}_{\\text{goal}} = \| \\boldsymbol{p}_{goal} - \\mathcal{P}^{real} \|^{2}

        See :func:`goalPositionErrorNorm` for details.
        """
        return self.goalPositionErrorNorm()**2

    def torquesNorm(self):
        """Calculate the :math:`l_{2}` norm of the joint torques.

        .. math::
            :label: torquesNorm

            \\boldsymbol{e} = \| \\mathcal{T} \|

        Here, :math:`\\mathcal{T}` is a matrix of joint torque vectors :math:`\\boldsymbol{\\tau}` at each timestep similar to the positions shown in :func:`positionErrorNorm`.
        """
        return np.linalg.norm(self.torques, ord=2, axis=1)

    def torquesSquaredNorm(self):
        """Calculate the squared :math:`l_{2}` norm of the joint torques.

        .. math::
            :label: torquesSquaredNorm

            \\boldsymbol{e}^{2} = \| \\mathcal{T} \|^{2}

        See :func:`torquesNorm` for details.
        """
        return self.torquesNorm()**2

    def torquesSquaredNormTimeAveraged(self):
        """Calculate the squared :math:`l_{2}` norm of the joint torques divided by the expected trajectory duration :math:`t_{\\text{end}}`.

        .. math::
            :label: torquesSquaredNormTimeAveraged

            \\boldsymbol{e}^{2} = \\frac{\| \\mathcal{T} \|^{2}}{t_{\\text{end}}}

        See :func:`torquesNorm` for details.
        """
        return self.torquesSquaredNorm() / self.duration

    def calculateGammaFactors(self):
        """Calculates the gamma scaling factor for the goal cost.

        The :math:`\\gamma` factor is calculated using:

        .. math::
            :label: gammaFactor

            \\boldsymbol{\\gamma} = \\left( \\frac{\\boldsymbol{t}}{t_{d}} \\right)^{\\beta}

        where :math:`\\boldsymbol{t}` are the individual timesteps, :math:`t_{d}` is the expected task duration, and :math:`\\beta` is a scaling factor (see :func:`setBetaFactor`).
        """
        return (self.time/self.expectedDuration)**self.beta

    def positionErrorSquaredNormTimeAveraged(self):
        return self.positionErrorSquaredNorm() / self.duration

    def goalPositionErrorSquaredNormPenalized(self):
        return self.gamma * self.goalPositionErrorSquaredNorm()

    def torquesSquaredNormTimeAveragedScaled(self):
        return self.torquesSquaredNormTimeAveraged() * self.energy_scaling_factor

    def trackingCost(self):
        return self.positionErrorSquaredNormTimeAveraged().sum()

    def goalCost(self):
        return self.goalPositionErrorSquaredNormPenalized().sum()

    def energyCost(self):
        return self.torquesSquaredNormTimeAveragedScaled().sum()

    def toString(self):
        s = "\n========================\n"+self.name+" Task Data:\n========================\n"
        s += "time.shape: " + str(self.time.shape) + "\n"
        s += "dt_vector.shape: " + str(self.dt_vector.shape) + "\n"
        s += "real.shape: " + str(self.real.shape) + "\n"
        s += "ref.shape: " + str(self.ref.shape) + "\n"
        s += "waypoints.shape: " + str(self.waypoints.shape) + "\n"
        s += "torques.shape: " + str(self.torques.shape) + "\n"
        s += "gamma.shape: " + str(self.gamma.shape) + "\n"
        s += "nTimeSteps: " + str(self.nTimeSteps) + "\n"
        s += "duration: " + str(self.duration) + "\n"
        s += "expectedDuration: " + str(self.expectedDuration) + "\n"
        s += "dt: " + str(self.dt) + "\n"
        s += "beta: " + str(self.beta) + "\n"
        s += "energy_scaling_factor: " + str(self.energy_scaling_factor) + "\n"
        s += "------ Cost Data ------\n"
        s += "trackingCost: " + str(self.trackingCost()) + "\n"
        s += "goalCost: " + str(self.goalCost()) + "\n"
        s += "energyCost: " + str(self.energyCost()) + "\n"
        s += "========================\n"
        return s

    def printData(self):
        print(self.toString())
