# Docker Image

To build the Docker images for Nvidia Jetson Xavier NX, you simply need to run `./docker_build.sh` on this folder, and it will build the images automatically. Make sure you setup SSH access to GitHub, as the submodules are linked through SSH (not HTTP).

After building the image, run `./docker_run.sh` to automatically start the Docker container.

To use ROS remotely (from your computer to the Jetson Container), make sure to follow the [Network Setup](http://wiki.ros.org/ROS/NetworkSetup) guide.


## Configure SSH connection to the Nvidia Jetson

Ensure that you have the the Github SSH keys set up in **your computer**, if not follow [this](https://techyarsal.medium.com/how-to-setup-git-the-proper-way-part-2-setting-up-ssh-key-ef745e5e8bfb) tutorial.

Open a terminal in the **Nvidia Jetson** and run the following command, replace the ``username`` by your Github username

```bash
  
  ssh-import-id gh:username
  
```

In **your computer** edit the config file 

```bash
  
  nano /home/$USER/.ssh/config
  
```
and add a new host by pasting the following text

```
Host name
  User jetson_user
  HostName jetson_hostname.local
  ForwardX11 yes
  ForwardAgent yes
  Port 22
 ```
 
 where the Host is the name you want, the User and the Hostname are the Jetson parameters.



## Commands to use the ROS remotely

### Jetson 

```bash

  export ROS_IP= `ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'` | awk '{print $2}'

```


### Computer


```bash

  export ROS_MASTER_URI='http://activespace-jetson.local:11311'
  
  export ROS_IP= `ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'` | awk '{print $2}'
 
```
