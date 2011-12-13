Installation der Pathcompare Plug-ins

1.) Installiere ROS Diamondback, siehe dazu:
http://www.ros.org/wiki/diamondback/Installation/Ubuntu


2.) Verschiebe den Ordner pathcompareplugins in das Verzeichnis: 
/opt/ros/diamondback/stacks/


3.) Wechsle in das Verzeichnis 
cd /opt/ros/diamondback/stacks/pathcompareplugins


4.) führe cmake aus:
cmake CMakeLists.txt

5.) starte Buildvorgang
make

