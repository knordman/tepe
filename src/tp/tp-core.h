
#pragma once

#ifndef TP_BODIES
#error application needs to define TP_BODIES to the number of dynamcial bodies
#define TP_BODIES 0
#endif

#ifndef TP_HINGES
#error application needs to define TP_HINGES to the number of hinges
#define TP_HINGES 0
#endif

#ifndef TP_MOTORS
#error application needs to define TP_MOTORS to the number of motors
#define TP_MOTORS 0
#endif

#ifndef TP_FEET
#error application needs to define TP_FEET to the number of feet
#define TP_FEET 0
#endif

#ifndef TP_TYPES
#include "types/default.h"
#else
#include TP_TYPES
#endif

#define TP_CONTACTS_ON_FOOT			3
#define TP_CONTACT_CONSTRAINTS		(TP_CONTACTS_ON_FOOT+2)
#define TP_CONSTRAINTS				(5*(TP_HINGES)+(TP_MOTORS)+TP_CONTACT_CONSTRAINTS*(TP_FEET))
#define TP_HINGE_CONSTRAINTS		(5*(TP_HINGES))
#define TP_HINGE_MOTOR_CONSTRAINTS	(5*(TP_HINGES)+(TP_MOTORS))

#define TP_REAL(X) ((real_t)(X))

#ifndef TP_ERP
#define TP_ERP TP_REAL(0.8)
#endif

#define TP_PI TP_REAL(3.1415926535)

#ifndef TP_MEM
#include "memory/simple.h"
#else
#include TP_MEM
#endif
#include "memory/memory.h"
#include "alglin.h"
