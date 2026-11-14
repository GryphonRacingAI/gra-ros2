import os
import re
import pycuda.driver as cuda
from pycuda.compiler import SourceModule
import pycuda.autoinit



def preprocess_module(module: str, args: dict, no_extern_c=False):
    '''Replaces special markers <<...>> in cuda source code with
       values provided in the args dictionary.

    '''
    def repl(match):
        contents = match.group(1)
        if contents in args:
            return str(args[contents])
        else:
            raise Exception("Failed to replace a placeholder in the source code")


    module = re.sub(r'<<([a-zA-Z][\w]*?)>>', repl, module)
    return SourceModule(module, no_extern_c=no_extern_c)


# Optional kernel parameters. The defaults reproduce the previous behaviour, so callers that do not
# pass them (simulation.py, fsonline.py, ins.py) keep working.
OPTIONAL_ARGS = {
    "FAR_RANGE": 1e9,   # measurements beyond this range (m) use FAR_VAR_R / FAR_VAR_B instead of the sensor covariance
    "FAR_VAR_R": 1.0,   # range variance (m^2) for far measurements
    "FAR_VAR_B": 1.0,   # bearing variance (rad^2) for far measurements
    "NEW_LM_D2": 0.0,   # > 0: a measurement that starts a new landmark scales the particle weight by the
                        # likelihood of a measurement at this squared Mahalanobis distance (0 = off)
    # Data association (update.cu, update_landmarks):
    "COLOR_MISMATCH_FACTOR": 1.0,  # association distance to a landmark of another type is multiplied by this.
                                   # 1 = type ignored (as before), > 1 = soft gate (a landmark of the wrong type
                                   # must be that much closer), 1e9 = hard gate (never matched)
    "COLOR_VOTE": 0,    # 1: a landmark's type is a vote. A matched measurement of another type takes one hit
                        # (4) off the existence score instead of adding it; when the score falls below the
                        # creation value (15) the landmark takes the measurement's type and restarts at 15.
                        # Needs COLOR_MISMATCH_FACTOR < 1e9, otherwise mismatches are never matched.
    "GATE_D2": 0.0,     # > 0: gate and choose by squared Mahalanobis distance in (range, bearing) with the
                        # innovation covariance H P H^T + R_i (R_i = the far covariance beyond FAR_RANGE), so the
                        # gate widens with the measurement's uncertainty; 9.21 = 99 % for 2 dof. 0 = Euclidean
                        # distance against THRESHOLD (as before).
}


def load_cuda_modules(**args):
    args = {**OPTIONAL_ARGS, **args}
    cu_dir = os.path.dirname(__file__)

    with open(os.path.join(cu_dir, "predict.cu"), "r") as f:                    # FIXED
        predict = f.read()

    with open(os.path.join(cu_dir, "update.cu"), "r") as f:                     # FIXED
        update = f.read()

    with open(os.path.join(cu_dir, "rescale.cu"), "r") as f:                    # FIXED
        rescale = f.read()

    with open(os.path.join(cu_dir, "resample.cu"), "r") as f:                   # FIXED
        resample = f.read()

    with open(os.path.join(cu_dir, "weights_and_mean_position.cu"), "r") as f:  # FIXED
        weights_and_mean = f.read()

    with open(os.path.join(cu_dir, "permute.cu"), "r") as f:                    # FIXED
        permute = f.read()

    return {
        "predict": preprocess_module(predict, args, no_extern_c=True),
        "update": preprocess_module(update, args),
        "rescale": preprocess_module(rescale, args),
        "resample": preprocess_module(resample, args),
        "weights_and_mean": preprocess_module(weights_and_mean, args),
        "permute": preprocess_module(permute, args)
    }
