# install docker
sudo apt-get update
sudo apt-get install -y docker.io

# add user to docker group permissions
sudo usermod -aG docker $USER