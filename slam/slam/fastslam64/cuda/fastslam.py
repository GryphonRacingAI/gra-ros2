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


    module = re.sub('<<([a-zA-Z][\w]*?)>>', repl, module)
    return SourceModule(module, no_extern_c=no_extern_c)


def load_cuda_modules(**args):
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
