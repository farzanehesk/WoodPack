## Install libraries

### open3d

	git clone https://github.com/isl-org/Open3D
	cd Open3D/
	cd util/
	./install_deps_ubuntu.sh
	cd ..
	mkdir build
	cd build/
	cmake ..
	make -j$(nproc)
	sudo make install





### VTK

	git clone https://gitlab.kitware.com/vtk/vtk.git
	cd vtk
	git checkout v9.1.0  # Match the version used in PCL
	mkdir build && cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DVTK_GROUP_ENABLE_MPI=YES
	make -j$(nproc)
	sudo make install