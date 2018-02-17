# Basic makefile
CXX=g++
SRCDIR=src
INCDIR=include
TESTDIR=test
BUILDDIR=build
PROG=$(BUILDDIR)/test_001 
OBJS=$(addprefix $(BUILDDIR)/,$(patsubst %.cpp,%.o,$(notdir $(wildcard $(SRCDIR)/*.cpp))))
TESTOBJS=$(addprefix $(BUILDDIR)/,$(patsubst %.cpp,%.o,$(notdir $(wildcard $(TESTDIR)/*.cpp))))
DEBUG=-g
DEPS=$(addprefix $(INCDIR)/,$(notdir$(wildcard $(INCDIR)/*.h))) 
CXXFLAGS=$(INCDIR:%=-I%) -Wall -c $(DEBUG)
LDFLAGS=-Wall $(DEBUG)

.PHONY: clean all

all: $(PROG)

$(PROG): $(OBJS) $(TESTOBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(BUILDDIR)/%.o:$(TESTDIR)/%.cpp $(DEPS) | $(BUILDDIR)
	$(CXX) -o $@ $< $(CXXFLAGS) -D LIB_TEST_MODE

$(BUILDDIR)/%.o:$(SRCDIR)/%.cpp $(DEPS) | $(BUILDDIR)
	$(CXX) -o $@ $< $(CXXFLAGS) -D LIB_TEST_MODE

$(BUILDDIR):
	@mkdir $@

clean:
	rm -f $(BUILDDIR)/*.o $(PROG)

run_test:
	@./$(PROG)
