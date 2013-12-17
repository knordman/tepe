
/** @defgroup tp-usage Settings and Usage
 *
 * Mandatory and tunable settings for TEPE.
 *
 */

/** @defgroup tp-alglin Linear Algebra
 *
 * Functions for vector and quaternion operations needed for the TEPE simulator.
 *
 * The linear algebra functions are designed to be used as inline functions
 * and are prefixed with the #TP_FUNC_INLINE
 * specifier, see \ref tp-types.
 */

/** @defgroup tp-dynamics Dynamics
 *
 * Functions that deal with defining a model, solving for constraint forces and integrating the simulation.
 *
 * Many of the functions in the Dynamics module are only used internally in TEPE, and is
 * not to be used by a user directly. The following functions are however "user-functions":
 * - set_box_inertia()
 * - set_cylinder_inertia()
 * - create_hinge()
 * - add_motor()
 * - hinge_angle()
 * - hinge_angle_rate()
 * - step_world()
 *
 * See \ref main-usage for more details how to setup a simulation.
 */

/** @defgroup tp-collision Collision Detection
 *
 * Functions to identify contact points.
 *
 * Currently, only a simple and specialized "tri foot" collision detection algorithm is included.
 * This means that contact points can only be identified when a body is collided as
 * a cylindrical foot towards a plane terrain.
 */

/** @defgroup tp-types Types
 *
 * Customizable types and function specifiers.
 *
 * The settings documented here correspond to the types/default.h settings. Another header
 * file may be used, by specifying it through the #TP_TYPES macro. An application might very well
 * benefit from creating its own type settings. Creating a platform unique file,
 * and e.g. add padding to arrays, might be necessary to fulfill alignment requirements.
 * Creating an application specific copy is also feasible when different
 * math functions should be used or function declarations need to have special
 * specifiers defined.
 *
 * A default (general CPU) setting and a CUDA setting are provided with TEPE.
 */

/** @defgroup tp-mem Memory
 *
 * Abstract memory layer to allow for specialized allocations. The storage of simulation data is
 * abstracted by a memory layer that allows for platform
 * independent storage implementations. From the application memory can be accessed
 * uniformly through the Memory Access Functions.
 *
 * A default memory allocation, and an allocation suitable for running the engine as a
 * CUDA kernel are provided. The macro #TP_MEM, is used for selecting the memory allocation, and
 * may link to a user-defined header. A memory implementation needs to provide a definition of
 * the struct mem_t type, in addition to implement all the Memory Access Implementation Unique Functions.
 */

/** @defgroup tp-dev Development
 *
 * Functions useful for development of TEPE.
 */

/** @defgroup tp-tests Unit tests
 *
 * Tests for TEPE using [CxxTest](http://cxxtest.com/). Linear algebra tests uses the
 * [Eigen matrix library](http://eigen.tuxfamily.org) for validating the results.
 */

