







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
	






### Phoxi Control Installation

1. Download the latest version of PhoXi Control and navigate to the Downloaded File

        www.photoneo.com/downloads/phoxi-control

2. Unpack the downloaded *.tar file

	    tar -xvf PhotoneoPhoXiControlInstaller-1.13.4-Ubuntu22-STABLE.tar.gz 

3. Make the Installer Executable

	    chmod +x PhotoneoPhoXiControlInstaller-1.13.4-Ubuntu22-STABLE.run

4. Run the Installer

	    sudo ./PhotoneoPhoXiControlInstaller-1.13.4-Ubuntu22-STABLE.run

5. The installation requires the user to accept EULA, to do this
automatically, pass the --accept flag to the installer script.

	    sudo ./PhotoneoPhoXiControlInstaller-1.13.4-Ubuntu22-STABLE.run --accept /opt/Photoneo/PhoXiControl-1.13.4/

6. Verify Installation and Reboot

        /opt/Photoneo/PhoXiControl-<version>

7. Restart your computer:

        sudo reboot

8. Run PhoXi Control

        PhoXiControl



### Collecting Log Files

1. Download the utility from 
    
        https://www.photoneo.com/dl/logdl-lin

2. Give the application executable rights (only needed once):
   
        chmod +x phoxi-log-downloader-0.0.2
   
4. Run the application using the camera’s IPv6 address (ID / IPv4 )

        ./phoxi-log-downloader-0.0.2 fe80::4ab0:2dff:fe55:edbe%enx606d3cfbccbf

5. Wait for the process to finish. If successful, you should see:

        Result: OK

6. Check the generated log file in the same folder:

        ls -l log.txt
