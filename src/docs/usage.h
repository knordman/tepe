/**
 * @name Mandatory Macro Definitions
 *
 * In order to use TEPE the simulation model must be defined at compile-
 * time. The definition of the following group of macros makes sure that enough
 * memory is allocated to hold the simulation model. These macros need to be defined
 * prior to the inclusion of tp.h.
 *
 */
//@{
/** \def TP_BODIES
 *
 * Should be set to the number of bodies.
 *
 * @ingroup tp-usage
 */
#define TP_BODIES

/** \def TP_HINGES
 *
 * Should be set to the number of hinges.
 *
 * @ingroup tp-usage
 */
#define TP_HINGES

/** \def TP_MOTORS
 *
 * Should be set to the number of motors.
 *
 * @ingroup tp-usage
 */
#define TP_MOTORS

/** \def TP_FEET
 *
 * Should be set to the number of bodies that act as feet.
 *
 * @ingroup tp-usage
 */
#define TP_FEET

//@}

/**
 * @name Tuning Macros
 *
 * These macros tune the behaviour of TEPE and need to be defined
 * before the inclusion of tp.h. No macro in this group is mandatory
 * to define.
 */
//@{
/** \def TP_TYPES
 *
 * Defines the type header to be used. The default setting is types/default.h.
 * TEPE also provides types/cuda.h, which are CUDA optimized type settings.
 * See \ref tp-types.
 *
 * @ingroup tp-usage
 */
#define TP_TYPES

/**
 * If the default type settings are used, this macro can be defined to set TEPE
 * to single precision.
 *
 * @ingroup tp-usage
 */
#define TP_DEFAULT_SINGLE

/**
 * If the default type settings are used, this macro can be defined to set TEPE
 * to double precision.
 *
 * @ingroup tp-usage
 */
#define TP_DEFAULT_DOUBLE

/** \def TP_ERP
 *
 * Defines the global error reduction parameter.Defaults to 0.8.
 *
 * @ingroup tp-usage
 */
#define TP_ERP

/** \def TP_MEM
 *
 * Defines the memory header/implementation to be used. The default setting is
 * to use memory/simple.h. TEPE also provides memory/cudaopt.h, which is an optimized
 * allocation for running multiple instances of TEPE, each in its own GPU thread.
 * See \ref tp-mem.
 *
 * @ingroup tp-usage
 */
#define TP_MEM
//@}

/**
 * @name Useful User Macros
 *
 * These macros are defined by TEPE when initialized. They are useful for various
 * purposes.
 */
//@{

/** \def TP_CONSTRAINTS
 *
 * Total number of constraints for which memory has been allocated.
 *
 * @ingroup tp-usage
 */
#define TP_CONSTRAINTS

/** \def TP_CONTACT_CONSTRAINTS
 *
 * Number of constraints that are allocated for each possible contact point.
 *
 * @ingroup tp-usage
 */
#define TP_CONTACT_CONSTRAINTS

/** \def TP_HINGE_CONSTRAINTS
 *
 * Index+1 for the last hinge constraint.
 *
 * @ingroup tp-usage
 */
#define TP_HINGE_CONSTRAINTS

/** \def TP_HINGE_MOTOR_CONSTRAINTS
 *
 * Index+1 for the last motor constraint.
 *
 * @ingroup tp-usage
 */
#define TP_HINGE_MOTOR_CONSTRAINTS

/** \def TP_REAL
 *
 * Macro to type cast a float to the configured precision.
 *
 * @ingroup tp-usage
 */
#define TP_REAL(X)

/** \def TP_PI
 *
 * Macro that defines the used value of \f$\pi\f$.
 *
 * @ingroup tp-usage
 */
#define TP_PI
//@}

/** \def TP_DEBUG
 *
 * Define this macro to activate debug functionality.
 *
 * @ingroup tp-dev
 */
#define TP_DEBUG
