# This makefile configures variables related to compilation, e.g. compiler flags.
# Furthermore, it defines compilation recipes
CFLAGS   := $(CFLAGS) -pthread -Wall
LFLAGS   := $(LFLAGS) -lpthread -pthread
CC       := g++
INC_DIR  := $(INC_DIR) -I$(SRCS_DIR) -I$(LIBS_DIR)
DEPFLAGS  = -MT $@ -MMD -MP -MF $(OBJS_DIR)/$*.d
# DEPFLAGS ensures that changes to headers trigger recompilation properly. More info on the wiki:
# https://github.com/Hyp-ed/hyped-2020/wiki/Makefiles#what-is-the-line-depflags----mt---mmd--mp--mf-objs_dird-in-buildmk

ifeq ($(RELEASE),1)
  CFLAGS += -O2
else
  CFLAGS += -Og
endif

ARCH := $(shell uname -m)
ifneq (,$(findstring 64,$(ARCH)))
  ARCH := 64
else
  ARCH := 32
endif

# Reconfigure from cross-compilation
ifeq ($(CROSS), 1)
  ARCH := 32

  ifeq ($(UNAME), Darwin)
    $(info cross-compiling using Mac host)
	CC := mac-crosscompiler/prebuilt/bin/clang++
	export COMPILER_PATH = mac-crosscompiler/sysroot/usr/lib/gcc/arm-linux-gnueabihf/6.3.0/
	CFLAGS := $(CFLAGS) --target=arm-linux-gnueabihf \
					    --sysroot=mac-crosscompiler/sysroot \
					    -isysroot mac-crosscompiler/sysroot \
					    -Imac-crosscompiler/sysroot/usr/include/c++/6.3.0 \
					    -Imac-crosscompiler/sysroot/usr/include/arm-linux-gnueabihf/c++/6.3.0 \
					    --gcc-toolchain=mac-crosscompiler/prebuilt/bin \
					    -DLINUX
	LFLAGS := $(LFLAGS) --target=arm-linux-gnueabihf -L$(COMPILER_PATH) --sysroot=mac-crosscompiler/sysroot
  else
    $(info cross-compiling using Linux host)
	CC := hyped-cross-g++
	LFLAGS := $(LFLAGS) -static
  endif
else
  ifeq ($(UNAME), Linux)
    CFLAGS += -DLINUX
  endif
endif

CFLAGS += -DARCH_$(ARCH)

# test if compiler is installed
ifeq ($(shell which $(CC)), )
	$(warning compiler $(CC) is not installed)
endif

# detect if clang is being used as compiler
USING_CLANG := $(shell $(CC) --version 2>&1 | grep -c "clang")

# only use -MJ flag to generate compilation database if using clang
ifeq ($(USING_CLANG), 1)
	COMPILATION_DB = -MJ $(addsuffix .json, $@)
endif

LL := $(CC)

# auto-discover all sources
SRCS      := $(shell find $(SRCS_DIR) -name '*.cpp')
OBJS      := $(patsubst $(SRCS_DIR)%.cpp,$(OBJS_DIR)%.o,$(SRCS))
MAIN_OBJ  := $(patsubst run/%.cpp, $(OBJS_DIR)/%.o, $(MAIN))

$(TARGET): $(DEPENDENCIES) $(OBJS) $(MAIN_OBJ)
	$(Echo) "Linking executable $(MAIN) into $@"
	$(Verb) $(LL)  -o $@ $(OBJS) $(MAIN_OBJ) $(LFLAGS) $(COVERAGE_FLAGS)
ifeq ($(USING_CLANG), 1)
# This finds all the individual compilation database files in the bin directory,
# and combines them into one compile_commands.json file
# Only works with gnu sed, couldn't get newlines to work with bsd sed
# On mac use homebrew to install gnu sed
# Double $ signs are so Make interprets it as a single regex $, on command line use a single $ sign
	$(Verb) find bin -name '*.json' -exec sed -e '1s/^/[\n/' -e '$$s/,$$/\n]/' {} + > compile_commands.json
endif

$(MAIN_OBJ): $(OBJS_DIR)/%.o: $(MAIN)
	$(Echo) "Compiling main: $<"
	$(Verb) mkdir -p $(dir $@)
	$(Verb) $(CC) $(DEPFLAGS) $(CFLAGS) $(COMPILATION_DB) -o $@ -c $(INC_DIR) $< $(COVERAGE_FLAGS)

$(OBJS): $(OBJS_DIR)/%.o: $(SRCS_DIR)/%.cpp
	$(Echo) "Compiling $<"
	$(Verb) mkdir -p $(dir $@)
	$(Verb) $(CC) $(DEPFLAGS) $(CFLAGS) $(COMPILATION_DB) -o $@ -c $(INC_DIR) $< $(COVERAGE_FLAGS)
