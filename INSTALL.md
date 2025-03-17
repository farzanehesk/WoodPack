



## Install libraries
### open3d



### PCL

### vtk

	git clone https://gitlab.kitware.com/vtk/vtk.git
	cd vtk
	git checkout v9.1.0  # Match the version used in PCL
	mkdir build && cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DVTK_GROUP_ENABLE_MPI=YES
	make -j$(nproc)
	sudo make install
	
	
	
#### trouble shooting
issues with aMPI and VTK
IN CMAKELISTS: YOU SHOULE ENABLE :
enable_language(C)
enable_language(CXX)



### Git
If you want to completely undo the commit (delete the commit itself):
	git reset --soft HEAD~1
	
