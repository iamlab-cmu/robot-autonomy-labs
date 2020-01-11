# Robot Autonomy | Labs

This repository contains code and instructions for running labs for the 16-662 Robot Autonomy class.

## Docker Installation

Docker is required for running robot control code from your personal computers.
For the class we'll only support running Docker on Ubuntu 16.04/18.04.

If you have a Mac or Windows 10 Pro (not Home), everything should still work as expected.

To install Docker:
- On Linux, run `bash docker/install_docker_linux.sh`
- On Windows, follows [these instructions](https://docs.docker.com/docker-for-windows/)
- On Mac, follows [these instructions](https://docs.docker.com/docker-for-mac/install/)

## Running Robot Client Docker Container

1. Get the latest robot-client docker image:

   `docker pull iamlab/robot-client:latest`

2. Run the launch script:

   `bash docker/run_robot_client_docker_container.sh <ROBOT_CLIENT_IO_PATH>`

   `ROBOT_CLIENT_IO_PATH` is an absolute path to a directory on your computer. This directory is mounted on the docker container at the location `/robot_client_io`

   This directory must contain a bash script called run.sh. The docker container will execute this run.sh after initialization. It is expected that whatever is being executed in run.sh will read from and write to the directory `/robot_client_io`, and nowhere else.

   For an example, to run the Lab 1 code, see the folder `lab1/robot_client_io` 
   
   To use this folder, run:

   `bash docker/run_robot_client_docker_container.sh $(pwd)/lab1/robot_client_io`
