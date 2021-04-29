# ank_ris
# Setting up the ank_ris package
First download the ank_ris_setup.sh file from the sh directory to your laptop.

Then run this script using: sh ank_ris_setup.sh.

This should download and setup our package as well as all it's dependencies.

# Running nodes in the ank_ris package
All the nodes in our package are designed to run from a laptop's terminal but runnig on the duckiebot's URI.

To make this possible, you export the ROS_MASTER_URI on every terminal you wish to use our package to the duckiebot's master URI using the command : export ROS_MASTER_URI=http://"name of bot".local:11311 

where "name of bot" will be replaced by ank or lead

The following nodes are the most rigorously tested and functional in our package:

steering_circles --- a node designed to run on ank's uri which will cause it to follow a single duckiebot placed in front of it

lead_lb --- a node designed to run on lead's uri which will cause it to follow a single person in front of it

lead_rand --- a node designed to run on lead's uri which will cause it to move in a random pattern (useful for demonstrating the steering_circles node)
