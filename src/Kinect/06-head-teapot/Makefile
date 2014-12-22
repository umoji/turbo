include ../build/common/CommonDefs.mak

BIN_DIR = ../bin

INC_DIRS = /usr/local/include/ni /usr/include/ni

SRC_FILES = ./*.cpp

EXE_NAME = 06-head-teapot

ifeq ("$(OSTYPE)","Darwin")
	LDFLAGS += -framework OpenGL -framework GLUT
else
	USED_LIBS += glut GL
endif

USED_LIBS += OpenNI

include ../build/common/CommonCppMakefile
