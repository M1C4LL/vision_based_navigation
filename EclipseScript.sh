# Script according to ROS IDEs chapter 2.3 catkin tools on 
# http://wiki.ros.org/IDEs#Building_the_project_inside_Eclipse
# To generate eclipse-project you need to execute: 

ROOT=$PWD 
cd build
for PROJECT in `find $PWD -name .project`; do
    DIR=`dirname $PROJECT`
    echo $DIR
    cd $DIR
    awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project
done
cd $ROOT
