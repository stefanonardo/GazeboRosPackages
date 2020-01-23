import numpy as np

from bases import MultiInputSecondOrderBases
from utils import TimeDelay

class Cerebellum:
    def __init__(self, dt, n_inputs, n_bases, beta, kc, delta):
        self.dt = dt
        self.beta = beta
        self.kc = kc
        self.delta = delta
        self.n_inputs = n_inputs
        
        # Initialize cortical bases
        self.tau_r = np.random.uniform(low=2.0, high=50.0, size=n_bases)*1e-3 # ms
        self.tau_d = np.random.uniform(low=50.0, high=750.0, size=n_bases)*1e-3 # ms
        self.p = MultiInputSecondOrderBases(self.dt, self.n_inputs, self.tau_r, self.tau_d)

        # Weights
        self.weights = np.zeros(n_inputs*n_bases)

        # Signals
        self.C = 0
        self.internal_error = 0

        # Time delays
        self.C_delta = TimeDelay(delay=self.delta, initial_value=self.C)
        self.p_delta = TimeDelay(delay=self.delta, initial_value=self.p.value)

    def step(self, CS, E):
        # Gives p_r, p_d and p at time t

        self.p.step(CS)

        # Update p time delay
        self.p_delta.step(self.p.value)

        # Output of microcircuit
        self.C = max(np.dot(self.p.value, self.weights), 0)
        # Update C time delay
        self.C_delta.step(self.C)

        # Update error signal and weights
        self._update_error_signal(E)
        self._update_weights(self.internal_error)

        return self.C

    def _update_error_signal(self, E):
        # IO not "spiking", just plain difference error
        self.internal_error = E - self.kc * self.C_delta.output

    def _update_weights(self, error):
        self.weights += self.beta * error * self.p_delta.output

    @property
    def output(self):
        return self.C

    def import_state(self, data):
        self.weights = data['weights']
        self.tau_r = data['tau_r']
        self.tau_d = data['tau_d']
        self.p = MultiInputSecondOrderBases(self.dt, self.n_inputs, self.tau_r, self.tau_d)

    def export_state(self):
        return {'weights': self.weights, 'tau_r': self.tau_r, 'tau_d': self.tau_d}


if __name__ == "__main__":
    """     mc = MicroCircuit(J=10, beta=0.001)
    print( mc.step(1, 0.2) )
    for i in range(50):
        print( mc.step(0, 0.2) ) """
    Ts = 1e-3
    c = Cerebellum(Ts, 2, 10, 0.001, 1, 20)
    print(c.step([1, 1], 0))
    
    for i in range(50):
        print( c.step([0, 0], 0.2) )