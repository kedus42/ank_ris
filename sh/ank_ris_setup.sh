sudo apt install git-all
pip install opencv-python
pip install PyQt5
roscd
cd ../src
catkin_create_pkg ank_ris
cd ank_ris
rm CMakeLists.txt package.xml
git init
git remote add origin https://github.com/kedus42/ank_ris
git pull origin
roscd
cd ..
catkin_make
source devel/setup.bash