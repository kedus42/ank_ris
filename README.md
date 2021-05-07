# Setting up the ank_ris package
First download our bash script that will download and setup the package for you:

```
wget https://raw.githubusercontent.com/kedus42/ank_ris/master/sh/ank_ris_setup.sh
```

Then run the downloaded script:
```
source ank_ris_setup.sh
```

Export the ros_master_uri of your laptop to the duckie container's uri. Change "name" in the command below to your duck's name:
```
export ROS_MASTER_URI=http://"name".local:11311
```

Then run the gui.launch file replacing "name" with the name of your duck. This will run all the nodes needed:
```
roslaunch ank_ris gui.launch duckie:="name"
```
