# -----------------------------------------------------------------------------
# Configuration options
# -----------------------------------------------------------------------------

# -g for debug info (gcc)
OPTIMIZATION = 			-O2

# SINGLE or DOUBLE
PRECISION = 				SINGLE

CXX = 					g++
CC = 					gcc

# -----------------------------------------------------------------------------
# Targets
# -----------------------------------------------------------------------------

PRGS :=		build/hinge				# Demo for hinge constraints
PRGS +=		build/contact 				# Demo for contact constraints

TESTS :=		build/alglin_unit
TESTS +=	build/computex_unit
TESTS +=	build/consolv_unit
TESTS +=	build/simplemem_unit

all		:	$(PRGS)
tests	:	$(TESTS)
docs	:	
	doxygen src/docs/Doxyfile
clean	:
	rm -rf build

.PHONY : clean all tests docs

# -----------------------------------------------------------------------------
# Set up flags according to the options above
# -----------------------------------------------------------------------------

CFLAGS := -Wall $(OPTIMIZATION)

ifeq ($(strip $(PRECISION)), SINGLE)
	CFLAGS += -DdSINGLE -DTP_DEFAULT_SINGLE
else
	CFLAGS += -DdDOUBLE -DTP_DEFAULT_DOUBLE
endif

INCLUDE_DIRS = 	-Isrc

CFLAGS += 		$(INCLUDE_DIRS)

LDFLAGS = 		-framework OpenGL -framework GLUT -lm -lpthread

LINK = 			$(CXX) $^ $(LDFLAGS) -o $@

DS_OBJS =		build/apps/common/util/drawstuff/drawstuff.o \
				build/apps/common/util/drawstuff/osx.o

TP_SRC =			$(shell find src/tp)

TP_UTIL_SRC =	$(TP_SRC) $(shell find src/apps/common/util -name *.h)

# -----------------------------------------------------------------------------
# Target specification
# -----------------------------------------------------------------------------

APP_OBJS =		$(DS_OBJS) \
				build/apps/common/util/configurable/Configurable.o \
				build/apps/common/util/parser/parser.o \
				build/apps/common/util/parser/parser-scanner.o

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

build/apps/hinge/main.o : $(TP_UTIL_SRC)

build/hinge : $(APP_OBJS) build/apps/hinge/main.o
	$(LINK)	

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

build/apps/contact/main.o : $(TP_UTIL_SRC)

build/contact : $(APP_OBJS) build/apps/contact/main.o 
	$(LINK)

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

# -----------------------------------------------------------------------------
# Implicit rules
# -----------------------------------------------------------------------------

build/%.o : src/%.cpp Makefile
	@mkdir -pv $(dir $@)
	$(CXX) $(CFLAGS) -c $< -o $@

build/%.o : src/%.c src/%.h Makefile
	@mkdir -pv $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o : src/%.cu Makefile
	@mkdir -pv $(dir $@)
	$(NVCC) $(NVCCCFLAGS) -c $< -o $@

src/%.cpp : src/%.y src/%.l
	bison -d $< -o $@

src/%-scanner.cpp : src/%.l
	flex -o $@ $< 
# -d to debug

build/%_unit : src/tests/%_test.h Makefile src/tests/helpers.h
	@mkdir -pv build/tests
	cxxtest/bin/cxxtestgen --error-printer -o build/tests/$*_runner.cpp $<
	$(CXX) -Wall \
		$(INCLUDE_DIRS) \
		-Icxxtest -I. \
		-c build/tests/$*_runner.cpp \
		-o build/tests/$*_runner.o
	$(CXX) $(OBJS) $(LIBS) -lm build/tests/$*_runner.o -o $@
