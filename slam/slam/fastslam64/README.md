# fastslam64 (vendored)

GPU FastSLAM 1.0 from https://github.com/GryphonRacingAI/fastslam64 at commit
`9020899e0dcd50417abf851c94d56a256e3c4157` (MIT, see `LICENSE`), reduced to what `fastslam_node.py`
uses: `lib/{particle3,common,utils}.py` and `cuda/*`. It was previously installed with pip and its
kernels patched by string replacement at start-up; the fixes are now in the source.

Changes to `cuda/update.cu`:
- EKF landmark update read its second input element from the (zero) output vector, so the bearing
  residual was dropped and landmarks were never corrected sideways.
- Data association only matches landmarks of the measured colour (UNKNOWN matches anything).
- Separate measurement covariance beyond `FAR_RANGE` (camera-only cones are much noisier than lidar).
- A measurement that starts a new landmark multiplies the particle weight by `NEW_LM_D2`'s
  new-feature likelihood, so a drifted particle that matches nothing no longer beats a good one.

Changes to `cuda/weights_and_mean_position.cu`:
- Heading mean is the weighted circular mean (the linear mean flipped by up to 180 deg near +-pi).

Placeholders `<<NAME>>` in the kernels are filled by `cuda.fastslam.load_cuda_modules(**args)`;
the node passes `THREADS PARTICLE_SIZE N_PARTICLES FAR_RANGE FAR_VAR_R FAR_VAR_B NEW_LM_D2`.
