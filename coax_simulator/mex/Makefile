CXXLIBS='-L/Applications/MATLAB_R2010b.app/bin/maci64 -lmx -lmex -lmat -lstdc++ -framework Accelerate -lgsl'

all: mexCoaXModel

mexCoaXModel: mexCoaXModel.cc CoaXModel.o
	$(MEX) -I`rospack find armadillo`/armadillo/include -I../include $(CXXLIBS) -L../lib $^ -output ../bin/$@

CoaXModel.o: ../src/CoaXModel.cc
	$(CXX) -I../include -I`rospack find armadillo`/armadillo/include  -c $^ -o $@

clean:
	rm -fr *.o *~