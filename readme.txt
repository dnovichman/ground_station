This is the ground station software for our PX4 customised avionics system
which can be cloned from
git@cvr-eng.cecs.anu.edu.au:juan.adarve/anu-avionics.git

It contains the following ros packages
control_input
lmpc_V1
vicon_bridge
mavlink_ros
log_data
phone

The description of these packages is found in Chapter 3 of my thesis.

Installation of the ground station is described in the wiki.

In gs_source/ground_station.h modify the config_file

To run ground station
"sh run_gs.sh"

The configuration file. 
ground_station.config

It can be modified to suit current parameters but the structure should not be 
changed.

To change the desired path or trajectory, go to the terminal running 
control_input

Examples of control_input terminal values are
0	Hover at (0.0 ,0.0, -1.1, 0.0) 
1	User specified differential flat output
4	Position (1.0, 0, -1.1, 0)
7	Figure 8 trajectory
17	Phone inputs

To enable wireless accesspoint on your ubuntu computer, android devices cannot detect ad-hoc hotspots created by following the standard hotspot created in Ubuntu. These steps in
http://askubuntu.com/questions/323335/how-to-setup-a-wi-fi-hotspot-with-an-ubuntu-laptop-access-point-mode 
by Kexanie should be followed.

git clone https://github.com/oblique/create_ap
cd create_ap
make install

Usage
No passphrase (open network):

sudo create_ap wlan0 eth0 MyAccessPoint

WPA + WPA2 passphrase:

create_ap wlan0 eth0 MyAccessPoint MyPassPhrase

Change wlan0 and eth0 to the interface on your machine. (ifconfig)


