# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs296_28_exe
IFOLDER = ~/distributable

PROJECT_ROOT := ./
EXTERNAL_ROOT=$(PROJECT_ROOT)external
SRCDIR = $(PROJECT_ROOT)src
OBJDIR = $(PROJECT_ROOT)myobjs
BINDIR = $(PROJECT_ROOT)mybins
DOCDIR = $(PROJECT_ROOT)doc
SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL 
NewLI = -lCS296test -lBox2D -lglui -lglut -lGLU -lGL 


CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
NewLD+=-L $(PROJECT_ROOT)mylibs -L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################
SHARED_LIB =TRUE
ifeq ($(SHARED_LIB) ,FALSE)
	decider=newtargeta
else
	decider=newtargetso
endif


.PHONY: setup all dummy doc
 
dummy: setup $(BINDIR)/$(TARGET) doc report


#exe: setup $(BINDIR)/$(TARGET)
#exelib: setup $(OBJS) $(decider)
#	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "cs296_28_exelib"
#	@if [ $(SHARED_LIB) == FALSE ];then $(CC) -o cs296_28_exelib $(NewLD) $(OBJDIR)/main.o $(NewLI);\
#	else $(CC) -o cs296_28_exelib $(NewLD) $(OBJDIR)/main.o $(NewLI); fi;
 	
#$(BINDIR)/$(TARGET)
#newtarget: ifeq ($(SHARED_LIB), TRUE)
#  	newtargeta 
#else 
#	newtargetso 
#endif
#newtargeta:
#	@cd $(OBJDIR);ar -cq libCS296test.a [!main]*.o;cd ..;mv $(OBJDIR)/libCS296test.a $(PROJECT_ROOT)mylibs/libCS296test.a;
#newtargetso:
#	@cd $(OBJDIR); gcc -shared -Wl,-soname,libCS296test.so -o libCS296test.so [!m]*.o;cd ..; mv $(OBJDIR)/libCS296test.so $(PROJECT_ROOT)mylibs/libCS296test.so;


-include $(OBJS:.o=.d)
$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp 
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

$(BINDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err


setup:
	@$(ECHO) "Setting up Compilation..."
	@mkdir -p myobjs
	@mkdir -p mybins
	@mkdir -p mylibs
	@$(ECHO) $(PROJECT_ROOT)
	@if [ -f $(PROJECT_ROOT)external/include/Box2D/Box2D.h ] && [ -f $(PROJECT_ROOT)external/lib/Box2D/Box2D.h ];\
	then $(ECHO) -n "Done...";\
	else cd $(PROJECT_ROOT)external/src; tar xzf ./Box2D.tgz; cd Box2D; cd Build; cmake ../; make install;\
	cd $(PROJECT_ROOT);\
	fi;
clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(OBJDIR) *~ $(DEPS) $(SRCDIR)/*~
	@$(ECHO) "Done"


distclean: clean
	@$(RM) -rf $(PROJECT_ROOT)mylibs $(BINDIR) $(DOCDIR)/doc $(DOCDIR)/*.aux $(DOCDIR)/*.bbl $(DOCDIR)/*.log $(DOCDIR)/*.blg $(DOCDIR)/*.pdf cs296_28_exelib $(PROJECT_ROOT)obj $(PROJECT_ROOT)bin $(PROJECT_ROOT)external/include/* $(PROJECT_ROOT)external/lib/* $(PROJECT_ROOT)external/src/Box2D

#report: ./report.tex plot
report: 
	@cd doc;pdflatex report;bibtex report;pdflatex report;bibtex report;pdflatex report;bibtex report;pdflatex report;pdflatex report;
doc:
	@cd $(DOCDIR);doxygen
install:
	@mkdir $(IFOLDER)
	@cp $(PROJECT_ROOT)mybins/cs296_28_exe $(IFOLDER)/cs296_28_exe
	@cp -r $(DOCDIR)/doc/html $(IFOLDER)/html
	@cp $(DOCDIR)/report.pdf $(IFOLDER)/report.pdf
dist: distclean
	@cd ..; tar cf cs296-g28-project.tar g28_project .git README.txt; gzip cs296-g28-project.tar; 
