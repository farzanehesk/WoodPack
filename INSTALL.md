



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



### PCL

	sudo apt update
	sudo apt install git build-essential cmake libeigen3-dev libflann-dev libboost-all-dev libvtk9-dev libqhull-dev libusb-1.0-0-dev libpcap-dev libpng-dev libjpeg-dev
	git clone https://github.com/PointCloudLibrary/pcl.git
	cd pcl
	mkdir build
	cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j$(nproc)
	sudo make install

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
	
