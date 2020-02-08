CFLAGS:=-pthread -O2 -Wall
LFLAGS:=-lpthread -pthread
COVERAGE_FLAGS=--coverage
OBJS_DEBUG_DIR:=bin/debug
SHELL:=/bin/bash
CC:="g++"
UNAME=$(shell uname)
ifneq ($(UNAME),Linux)
	# assume Windows
	UNAME='Windows'
	CFLAGS:=$(CFLAGS) -std=gnu++11
else
	CFLAGS:=$(CFLAGS) -std=c++11
endif

ROOT=$(shell git rev-parse --show-toplevel)

## Is 1 when python is installed
PYTHONCHECK=$(shell python2.7 -V  >/dev/null 2>&1 | grep -q 'Python 2.7(.[0-9]+)*'  | echo '1' )

# Manual override for LINTER
NOLINT=0

# Checks manual override and python flag defined in config
RUNLINTER=$(shell [[ $(NOLINT) == 0 && $(PYTHONCHECK) == 1 ]] && echo 1 )