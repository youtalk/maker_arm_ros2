import os

# The plant renders headless. MuJoCo picks its GL backend when it is first imported, so the
# default goes here, before any module of this package imports mujoco.
os.environ.setdefault('MUJOCO_GL', 'egl')
