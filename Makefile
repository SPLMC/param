###############################################
# global Makefile with automatic dependencies #
###############################################

######################
# external libraries #
######################

LIB    = -Wl,-Bstatic -lginac -lcln -lgmpxx -lgmp -lboost_program_options \
	-lluajit-5.1 -Wl,-Bdynamic -ldl

##################
# compiler flags #
##################

INCLUDE_DIRS = -I.

FLAGS        = -Wall -O3 -w -DNDEBUG -pipe $(INCLUDE_DIRS)
# FLAGS        = -Wall -Werror -O0 -ggdb -pipe $(INCLUDE_DIRS)

CXXFLAGS     = $(FLAGS)

################
# object files #
################

DIRS     := prismparser model2x rationalFunction parametric

OBJ_PRISMPARSER := prismparser/Expr.o   \
	    prismparser/AST.o           \
	    prismparser/PRISMParser.o   \
	    prismparser/PRISM.tab.o     \
	    prismparser/PRISM.yy.o      \
	    prismparser/Model.o         \
	    prismparser/Node.o          \
	    prismparser/Property.o      \
	    prismparser/Util.o          \
	    prismparser/Substitutor.o   \

OBJ_MODEL2X := 	model2x/Model2X.o \
	model2x/expr2lua.o        \
	model2x/Model2XError.o    \
	model2x/ValueCompute.o    \
	model2x/InitStatesComp.o

OBJ_RATIONALFUNCTION := rationalFunction/Polynomial.o \
	rationalFunction/Geobucket.o \
	rationalFunction/RationalFunction.o \
	rationalFunction/RationalCmp.o \
	rationalFunction/Base.o \
	rationalFunction/ExprConverter.o \
	rationalFunction/Cancellator.o

OBJ_PARAMETRIC := parametric/param.o \
	parametric/ModelExplorer.o \
	parametric/Model2XExplorer.o \
	parametric/LowLevelExplorer.o \
	parametric/RationalParser.o \
	parametric/ProgramOptions.o \
	parametric/Controller.o \
	parametric/Statistics.o \
	parametric/MayChange.o \
	parametric/StrongRefiner.o \
	parametric/Partition.o \
	parametric/Quotient.o \
	parametric/WeakRefiner.o \
	parametric/Refiner.o \
	parametric/Eliminator.o \
	parametric/BoundedIterator.o \
	parametric/ResultExporter.o \
	parametric/ModelExporter.o \
	parametric/Timer.o \
	parametric/PMM.o \
	parametric/PMC.o \
	parametric/SPMC.o \
	parametric/GPMC.o \
	parametric/PMDP.o \
	parametric/SPMDP.o \
	parametric/ExprToNumber.o \
	parametric/ModelChecker.o \
	parametric/IneqChecker.o \
	parametric/SparseNonDet.o \
	parametric/RegionsTODO.o \
	parametric/RegionResult.o \
	parametric/Cache.o \
	parametric/MC.o \
	parametric/MDP.o \
	parametric/MDPIterator.o

OBJ := $(OBJ_PRISMPARSER) $(OBJ_RATIONALFUNCTION) $(OBJ_PARAMETRIC) $(OBJ_MODEL2X)

####################
# build executable #
####################
param: $(OBJ)
	$(CXX) $(FLAGS) -o param $(OBJ) $(LIB)

###############
# build lexer #
###############
prismparser/PRISM.yy.cpp : prismparser/PRISM.l prismparser/PRISM.tab.hpp
	cd prismparser;\
	flex -PPRISM -oPRISM.yy.cpp PRISM.l

################
# build parser #
################
prismarser/PRISM.tab.hpp prismparser/PRISM.tab.cpp :	prismparser/PRISM.ypp
	cd prismparser;\
	bison -p PRISM -b PRISM -d -t PRISM.ypp

###########
# cleanup #
###########
clean:
	@for obj in $(OBJ); do \
		(if [ -e $$obj ]; then rm $$obj; fi) \
		done
	@for dir in $(DIRS); do \
		(cd $$dir; \
		 rm -rf *.o lang/parser.tab.{hpp,cpp} lang/lex.yy.c .*.d *~;\
		 cd ..) \
		done

######################
# build dependencies #
######################
Makefile.dep:
	@if [ ! -e Makefile.dep ]; then echo "# automatic dependencies" > Makefile.dep; fi
	@makedepend -w -a -Y -fMakefile.dep -- $(FLAGS) $(INCLUDE_DIRS) -- main.cpp &> /dev/null
	@for dir in $(DIRS); do \
		(makedepend -w -a -Y -fMakefile.dep -- $(FLAGS) $(INCLUDE_DIRS) -- $$dir/*.hpp $$dir/*.h $$dir/*.cpp) \
	done &> /dev/null

depend:
	@if [ -e Makefile.dep ]; then rm Makefile.dep; fi
	make Makefile.dep

########################################
# automatically generated dependencies #
########################################
include Makefile.dep
