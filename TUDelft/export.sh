echo "Warning: Did you commit first?"
echo "Cleanup TUDelft Paparazzi Version"
make clean
rm -vrf ./conf/conf.xml.*-*
echo "Cleaning (Not updateing) ENAC Paparazzi Version"
cd ../paparazzi3/
svn revert -R ./
svn status | sed 's/?   /rm -rf /' > rm.sh
chmod +x ./rm.sh
./rm.sh
cd ../ppz2svn/
echo "Exporting TUDelft To Paparazzi"
svn export ./ ../paparazzi3/ --force
cd ../paparazzi3/
svn status | grep 'TUDelft' | sed 's/?     /rm -rf /' > rm.sh
chmod +x ./rm.sh
./rm.sh
rm -rf ./import.sh
rm -rf ./export.sh
rm -rf ./sw/airborne/modules/onboardcam
rm -rf ./sw/airborne/modules/opticflow
rm -rf ./sw/airborne/boards/tiny_sense.h
rm -rf ./sw/airborne/booz/aerovinci
rm -rf ./conf/simulator/jsbsim/aircraft/LISA_BOOZ_BART.xml
echo "Check this list manually: what files need to be added to ENAC server?" > ./add.sh
svn status | sed 's/?       /svn add /' >> add.sh
chmod +x ./add.sh
gedit ./add.sh
