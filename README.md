http://mhernando.github.com/MRCore/


LINUX

 1. Go to your GITHUB directory i.e.: 

        cd ~/GITHUB
    
 2. Clone the repo:

        git clone https://github.com/mhernando/MRCore.git

 3. if you dont have a build directory, create it:

        mkdir -p BUILDS/MRCORE

 4. Move to the build folder

        cd BUILDS/MRCORE

 5. Use Cmake to configure the project:

        cmake ../../MRCore

 6. Compile MRCore

        cmake --build .

 7. Install it! 

        sudo make install

 8. if you want to compile the examples:

        cmake ../../MRCore -DCREATE_EXAMPLES=ON
        cmake --build .  
    









