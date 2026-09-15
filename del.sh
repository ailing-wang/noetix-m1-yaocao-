rm -rf .git
rm -rf src 
rm -rf include
cd build
sudo rm -rf CMakeFiles
sudo rm -rf logs
find . -type f ! -executable -exec rm -f {} +
