import pycuda.driver as cuda
import numpy as np

DOUBLE = 8


class CUDAMemory:
    def __init__(self, config):
        self.particles = cuda.mem_alloc(DOUBLE * config.N * config.PARTICLE_SIZE)

        self.scratchpad_block_size = 2 * config.N * config.MAX_LANDMARKS
        self.scratchpad = cuda.mem_alloc(DOUBLE * self.scratchpad_block_size)

        self.measurements = cuda.mem_alloc(DOUBLE * 2 * config.sensor.MAX_MEASUREMENTS)
        self.weights = cuda.mem_alloc(DOUBLE * config.N)
        self.ancestors = cuda.mem_alloc(DOUBLE * config.N)
        self.ancestors_aux = cuda.mem_alloc(DOUBLE * config.N)
        self.rescale_sum = cuda.mem_alloc(DOUBLE)
        self.cov = cuda.mem_alloc(DOUBLE * 4)
        self.mean_position = cuda.mem_alloc(DOUBLE * 3)
        self.cumsum = cuda.mem_alloc(DOUBLE * config.N)
        self.c = cuda.mem_alloc(DOUBLE * config.N)
        self.d = cuda.mem_alloc(DOUBLE * config.N)


    def free(self):
        self.particles.free()
        self.scratchpad.free()
        self.measurements.free()
        self.weights.free()
        self.ancestors.free()
        self.ancestors_aux.free()
        self.rescale_sum.free()
        self.cov.free()
        self.mean_position.free()
        self.cumsum.free()
        self.c.free()
        self.d.free()



def get_pose_estimate(modules, config, memory: CUDAMemory):
    estimate = np.zeros(3, dtype=np.float64)

    modules["weights_and_mean"].get_function("get_mean_position")(
        memory.particles, memory.mean_position,
        block=(config.THREADS, 1, 1)
    )

    cuda.memcpy_dtoh(estimate, memory.mean_position)
    return estimate


def rescale(modules, config, memory: CUDAMemory):
    modules["rescale"].get_function("sum_weights")(
        memory.particles, memory.rescale_sum,
        block=(config.THREADS, 1, 1)
    )

    modules["rescale"].get_function("divide_weights")(
        memory.particles, memory.rescale_sum,
        block=(config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1)
    )


def resample(modules, config, weights, memory: CUDAMemory, rand):
    cumsum = np.cumsum(weights)

    cuda.memcpy_htod(memory.cumsum, cumsum)

    modules["resample"].get_function("systematic_resample")(
        memory.weights, memory.cumsum, np.float64(rand), memory.ancestors,
        block=(config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1)
    )

    modules["permute"].get_function("reset")(memory.d, np.int32(config.N), block=(
        config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1))
    modules["permute"].get_function("prepermute")(memory.ancestors, memory.d, block=(
        config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1))
    modules["permute"].get_function("permute")(memory.ancestors, memory.c, memory.d, np.int32(
        config.N), block=(config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1))
    modules["permute"].get_function("write_to_c")(memory.ancestors, memory.c, memory.d, block=(
        config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1))

    modules["resample"].get_function("copy_inplace")(
        memory.particles, memory.c,
        block=(config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1)
    )

    modules["resample"].get_function("reset_weights")(
        memory.particles,
        block=(config.THREADS, 1, 1), grid=(config.N//config.THREADS, 1, 1)
    )
