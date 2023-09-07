#!/bin/bash

# mkdir Assets/Envs & mkdir Assets/Robots
mkdir -p Assets/Envs && mkdir -p Assets/Robots

# Download assets from google drive
Hostpial=https://www.dropbox.com/scl/fi/wq10pkpu1ctum1n7ex2hd/Hospital.zip?rlkey=4ol28zyk0ui238punvjeuqzr8
Unitree=https://www.dropbox.com/scl/fi/x61v4iboxihebii2rvug4/Unitree.zip?rlkey=jfiir19g2g6g0exvbajd1lq4q
Carter=https://www.dropbox.com/scl/fi/6ap70bvr8waf3hmg08ynw/Carter.zip?rlkey=8nydo8idvlfl2l2emtscv6wvf

wget -O Assets/Envs/Hospital.zip "$Hostpial"
wget -O Assets/Robots/Unitree.zip "$Unitree"
wget -O Assets/Robots/Carter.zip "$Carter"

# unzip files and remove .zip files
unzip Assets/Envs/Hospital.zip -d Assets/Envs && rm Assets/Envs/Hospital.zip
unzip Assets/Robots/Unitree.zip -d Assets/Robots && rm Assets/Robots/Unitree.zip
unzip Assets/Robots/Carter.zip -d Assets/Robots && rm Assets/Robots/Carter.zip