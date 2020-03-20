import numpy as np

from bases import MultiInputSecondOrderBases
from utils import TimeDelay

class Cerebellum:
    """ Adaptive filter model of the cerebellum 
    
        Parameters
        ----------
        `dt`: float
            Time step
        `n_inputs`: int
            Number of inputs to the cerebellar microcircuit
        `n_bases`: int
            Number of filters in each filter bank
        `beta`: float
            Learning rate.
        `kc`: float
            Strength of the NOI (nuclei-olivary inhibition)
        `delta`: int
            Delay between output and error signal given in units of `dt`.
    """
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
        """ Step forward by time `dt`.
            
            Parameters
            ----------
            `CS`: numpy.ndarray
                Contextual input (conditioned stimuli). Dimension `n_inputs`from constructor.
            `E`: float
                Error signal (uncoditioned stimuli).
        """
        self.p.step(CS)

        # Update p time delay
        self.p_delta.step(self.p.value)

        # Output of microcircuit. Linear combination of filter outputs, rectified to only give positive values.
        self.C = max(np.dot(self.p.value, self.weights), 0)

        # Update C time delay
        self.C_delta.step(self.C)

        # Update error signal and weights
        self._update_error_signal(E)
        self._update_weights(self.internal_error)

        return self.C

    def _update_error_signal(self, E):
        """ Computes the internal error signal. 

            The error is added with the inhibition from the output (NOI).

            Parameters
            ----------
            `E`: float
                Single-valued error signal.
        """
        self.internal_error = E - self.kc * self.C_delta.output

    def _update_weights(self, error):
        """ Updates the weights of the cerebellum. 

            Parameters
            ----------
            `error`: float
                Single-valued internal error signal.
        """
        self.weights += self.beta * error * self.p_delta.output

    @property
    def output(self):
        """ Ouput signal. Single valued. """
        return self.C

    def import_state(self, data):
        """ Imports a previously exported cerebellum model. Does not import the filter states. 
        
            Parameters
            ----------
            `data`: dict
                A dictionary with the following keys:
                    `weights`: numpy.ndarray
                        asdad
                    `tau_r`: numpy.ndarray
                        First time constants of the filters in each bank. Dimension equal to number of filters in each bank.
                    `tau_d`: numpy.ndarray
                        First time constants of the filters in each bank. Dimension equal to `tau_r`.
        """
        self.weights = data['weights']
        self.tau_r = data['tau_r']
        self.tau_d = data['tau_d']
        self.p = MultiInputSecondOrderBases(self.dt, self.n_inputs, self.tau_r, self.tau_d)

    def export_state(self):
        """ Exports the current cerebellum model. See `import_state` for a description of the data. """
        return {'weights': self.weights, 'tau_r': self.tau_r, 'tau_d': self.tau_d}


if __name__ == "__main__":"
    Ts = 1e-3
    c = Cerebellum(Ts, 2, 10, 0.001, 1, 20)
    print(c.step([1, 1], 0))
    
    for i in range(50):
        print( c.step([0, 0], 0.2) )