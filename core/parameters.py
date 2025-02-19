from dataclasses import dataclass
from typing import Optional

"""
Data class containing physics engine parameters for PyBullet simulation.
All parameters are optional with default values matching PyBullet's defaults.
This is used as a dataclass to allow easy modification of parameters.
"""

@dataclass
class TimeSteppingParams:
    num_substeps: int = 1
    """
    Number of substeps per simulation step.
    Subdivides the physics simulation step for better accuracy.
    Trades performance for accuracy.
    Default: 1 substep
    """
    fixed_timestep: float = 1.0 / 240.0
    """
    Physics engine timestep in seconds.
    Each call to 'stepSimulation' will progress simulation by this amount.
    Default: 1/240 seconds. Warning: Changing this can affect simulation stability.
    """

@dataclass
class SolverParameters:
    num_solver_iterations: int = 50
    """
    Maximum number of constraint solver iterations.
    Solver may terminate earlier if solverResidualThreshold is reached.
    Higher values increase accuracy but decrease performance.
    Default: 50 iterations
    """

    solver_residual_threshold: float = 1e-7
    """
    Velocity error threshold for constraint solver.
    If maximum velocity-level error is below this, solver terminates.
    Only applies if solver hasn't hit numSolverIterations.
    Default: 1e-7
    """

    warm_starting_factor: float = 0.85
    """
    Factor of previous-frame force/impulse used to initialize solver.
    Range: 0.0 to 1.0. Higher values can improve convergence.
    Default: 0.85
    """

    constraint_solver_type: int = 0
    """
    Type of constraint solver to use.
    Experimental feature - best to use default.
    Allows use of direct LCP solver like Dantzig.
    Default: 0 (standard solver)
    """

    global_cfm: float = 0.0
    """
    Global Constraint Force Mixing parameter.
    Experimental - controls constraint force mixing globally.
    Default: 0.0
    """

    minimum_solver_island_size: int = 1000
    """
    Minimum size of constraint solving islands.
    Experimental - affects how constraints are grouped.
    Avoids very small islands of independent constraints.
    Default: 1000
    """

@dataclass
class CollisionParameters:
    # Collision parameters
    use_split_impulse: int = 1
    """
    Enable split impulse feature.
    Advanced feature for maximal coordinates only.
    Splits position and velocity constraint solving to prevent penetration.
    Default: 1 (enabled)
    """

    split_impulse_penetration_threshold: float = 0.01
    """
    Threshold for split impulse penetration handling.
    If penetration is below this, no split impulse occurs.
    Units: meters
    Default: 0.01
    """

    collision_filter_mode: int = 0
    """
    Mode for collision filtering between groups.
    0: Default AND filter: (group A&maskB) AND (groupB&maskA)
    1: OR filter: (group A&maskB) OR (groupB&maskA)
    Default: 0
    """

    contact_breaking_threshold: float = 0.02
    """
    Distance threshold for breaking contacts.
    Contacts exceeding this aren't processed by solver.
    Also affects AABB extension.
    Units: meters
    Default: 0.02
    """

    contact_slop: float = 0.001
    """
    Position correction threshold for contacts.
    Contacts below this aren't resolved, improving stability.
    Units: meters
    Default: 0.001
    """

    enable_sat: int = 0
    """
    Enable Separating Axis Theorem for collision detection.
    Requires URDF_INITIALIZE_SAT_FEATURES in loadURDF.
    Alternative to GJK/EPA algorithms.
    Default: 0 (disabled)
    """

    allowed_ccd_penetration: float = 0.0
    """
    Continuous Collision Detection penetration threshold.
    CCD ignored if penetration below this value.
    Units: meters
    Default: 0.0
    """

@dataclass
class ErrorReductionParameters:
    # Error reduction parameters
    erp: float = 0.2
    """
    Error Reduction Parameter for non-contact constraints.
    Controls how aggressively constraint errors are resolved.
    Range: 0.0 to 1.0
    Default: 0.2
    """

    contact_erp: float = 0.2
    """
    Error Reduction Parameter for contacts.
    Controls how aggressively contact constraint errors are resolved.
    Range: 0.0 to 1.0
    Default: 0.2
    """

    friction_erp: float = 0.2
    """
    Error Reduction Parameter for friction.
    Only used when positional friction anchors are enabled.
    Range: 0.0 to 1.0
    Default: 0.2
    """

@dataclass
class FrictionParameters:
    # Friction parameters
    enable_cone_friction: int = 1
    """
    Toggle between cone and pyramid friction models.
    1: Use implicit cone friction (default)
    0: Use pyramid approximation
    Note: Try disabling if friction artifacts occur
    Default: 1
    """

@dataclass
class PerformanceParameters:
    max_num_cmd_per_1ms: int = 0
    """
    Limit commands per millisecond.
    Experimental: Adds 1ms sleep if exceeded.
    0: No limit
    Default: 0
    """

    enable_file_caching: int = 1
    """
    Enable caching of loaded files.
    Affects .obj wavefront file loading.
    Default: 1 (enabled)
    """
    pass

@dataclass
class PhysicsBehaviorParameters:
    # Physics behavior
    restitution_velocity_threshold: float = 0.2
    """
    Minimum velocity threshold for restitution.
    Restitution is zero below this relative velocity.
    Units: meters/second
    Default: 0.2
    """

    deterministic_overlapping_pairs: int = 1
    """
    Enable deterministic sorting of overlapping pairs.
    Affects backwards compatibility.
    Default: 1 (enabled)
    """

    joint_feedback_mode: int = 0
    """
    Coordinate frame for joint feedback.
    Options:
    - JOINT_FEEDBACK_IN_WORLD_SPACE
    - JOINT_FEEDBACK_IN_JOINT_FRAME
    Default: 0
    """

    report_solver_analytics: int = 0
    """
    Enable reporting of solver analytics.
    Provides additional solver performance data.
    Default: 0 (disabled)
    """
