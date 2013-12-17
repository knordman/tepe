Documentation {#mainpage}
==================

Purpose {#main-purpose}
==================

TEPE is a lightweight physics engine. It particularly has a compile-time defined 
and somewhat minimal memory usage. It was designed to run as a GPU kernel, enabling
numerous concurrent simulator instances to be set up (by for example 
using CUDA from Nvidia). TEPE is completely designed as a collection of inline functions 
in a set of header files. It uses the well known Projected Gauss-Seidel iterative solver.
This ensures small memory requirements, and can be implemented so that GPU thread divergence
is avoided, even when simulation worlds contain different amounts of constraints. 

In its current state it is most suitable for simulating mobile walking robots.
Currently the only supported constraint is the hinge joint which can have a
motor attached. There is no general collision detection, instead there is a 
simple "foot collision", where certain bodies tagged as "feet" are collided against
a plane. 

Usage {#main-usage}
==================

In order to use TEPE, initiate the engine in an application by first at least defining 
#TP_BODIES, #TP_HINGES, #TP_MOTORS and #TP_FEET. In addition to these mandatory
definitions there are also some macros that tune the simulator behaviour, see 
\ref tp-usage. After defining the necessary macros the simulator is available
by including tp.h. For two bodies, connected by a motor powered hinge which have
no contact with the ground one should use:
\code{.c}
		#define TP_BODIES		2
		#define TP_HINGES		1
		#define TP_MOTORS		1
		#define TP_FEET			0
		#include <tp/tp.h>
\endcode
Using the above setup lines, a default memory layout and a default type setting
are used. These can easily be changed by defining the #TP_MEM and #TP_TYPES macros. For
information on memory access and type settings, see \ref tp-mem and \ref tp-types. 
These sections provide information on how to implement custom memory layouts and 
type settings. TEPE includes besides the default settings also
settings that are useful for the Nvidia CUDA environment. To use these in the previous
example, setup TEPE as:
\code{.c}
		#define TP_BODIES		2
		#define TP_HINGES		1
		#define TP_MOTORS		1
		#define TP_FEET			0
		
		#define TP_MEM 			"memory/cudaopt.h"
		#define TP_TYPES 		"types/cuda.h"

		#include <tp/tp.h>
\endcode
Once the "setup" of TEPE is completed, one should allocate the simulation world memory.
One could have all the memory on the stack or dynamically allocate it, the functions
operating on the memory will always take a pointer to the memory structure. Also worth
noting is that the exact definition of the structure will vary between different memory
allocation layouts. 
\code{.c}
		struct mem_t *memory_pointer = malloc(sizeof(struct mem_t));
\endcode
After the memory has been allocated it is time to define the inertia of all the bodies in
the simulation world. This can be done by using the memory access methods described in
\ref tp-mem. For instance, to set the inertia tensor (or actually the inverse in the body frame) 
of the first body, one first declare and initialize a local matrix to hold the values,
\code{.c}
		tp_mtx inertia;
		inertia[0] = ...
		...
		inertia[1*TP_SIZE_VEC3] = ...
		...
		inertia[2*TP_SIZE_VEC3+2] = ...
\endcode
and then store it into the simulation memory, in the place of the first body,
\code{.c}
		set_mtx33(inertia, Ibi(memory_pointer, 0));
\endcode
See \ref tp-mem for more on how to access memory. For boxes and cylinders, TP include a pair of 
convinience functions for setting the inertia, see \ref set_box_inertia and \ref set_cylinder_inertia. 

Hinges are setup between bodies using the \ref create_hinge function while a motor is 
added to a hinge by using the \ref add_motor function. 

Once the world is configured, the simulation loop can be run as follows. If there are any
"feet" present these can be collided to the ground by using a loop similar to:
\code{.c}
		int contacts_offset = 0;		
		for(int foot = 0; foot < (TP_FEET); ++foot)
		{
			contacts_offset += collide_foot_cylinder_tri(
									memory_pointer,
									foot_radius_m,
									foot_height_m,
									contacts_offset,
									feet_index[foot]);
		}
\endcode
Here @a feet_index is an array that holds the index of the bodies that are considered to be"feet". The above 
code snippet will configure the necessary constraints in the case a contact is found. External forces should also be 
added before stepping the world. To add gravity to all bodies:
\code{.c}
		for(int body = 0; body < (TP_BODIES); ++body) 
			*z(tFe(memory_pointer, body)) += -9.81*(1.0/_mi(memory_pointer, body));
\endcode
The final step is to step the world:
\code{.c}
		step_world(memory_pointer, dt, iterations);
\endcode
Development {#main-dev}
=========================
When modifying TEPE there is a convinient debug header that provides some functions
to printout vectors and memory. To enable this, configure TEPE, by defining the #TP_DEBUG
macro and prepending the tp.h include by tp-core.h and debugging.h. 
\code{.c}
		#define TP_BODIES		2
		#define TP_HINGES		1
		#define TP_MOTORS		1
		#define TP_FEET			0
		
		#define TP_DEBUG
		
		#include <tp/tp-core.h>
		#include <tp/debugging.h>
		#include <tp/tp.h>
\endcode
