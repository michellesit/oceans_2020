# Oceans Docker
Creates a docker image to run this simulation in

## Creating Image
```
sudo docker build -t oceans20_demo:mar24 .

```

Note: This docker assumes the oceans_2020 repo is mounted on your local computer. Uncomment the install_uuv_pkgs.sh lines in the Dockerfile to automatically have it installed in the docker container (not recommended). OR run the same script on your local computer to have it install all the packages and folders as needed (recommended).

## Running Docker
 - **run_interactive_docker** - creates docker container and allows for gazebo/ros visualizations to appear. Requires two commandline arguments (docker imageID, docker nickname)
 - **attach_into_docker.sh** - ssh into your above running docker container. Useful for running multiple terminal windows.

 To use:
 ```
sudo chmod +x run_interactive_docker.sh attach_into_docker.sh
sudo docker images
```

Find the oceans20_demo docker imageID:
```
REPOSITORY                              TAG                 IMAGE ID            CREATED             SIZE
test_oceans                             mar23               fdd193ad7b39        23 hours ago        10.5GB
```

```
./run_interactive_docker.sh fdd193ad7b39 run_demo
```

In a different window:
```
./attach_into_docker.sh
```