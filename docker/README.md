# Docker Image

To build the Docker images for Nvidia Jetson Xavier NX, you simply need to run `./docker_build.sh` on this folder, and it will build the images automatically. Make sure you setup SSH access to GitHub, as the submodules are linked through SSH (not HTTP).

After building the image, run `./docker_run.sh` to automatically start a Docker container.

## Configure SSH connection to the Nvidia Jetson

Ensure that you have the the Github SSH keys set up in **your computer**, if not follow [this](https://techyarsal.medium.com/how-to-setup-git-the-proper-way-part-2-setting-up-ssh-key-ef745e5e8bfb) tutorial.

then open a terminal in the **Nvidia Jetson** and run the following command, replace the ``username`` by your Github username

```bash
  
  ssh-import-id gh:username
  
```

In **your computer** edit the config file 

```bash
  
  nano /home/$USER/.ssh/config
  
```
and add a new host

```
Host name
  User jetson_user
  HostName jetson_hostname.local
  ForwardX11 yes
  ForwardAgent yes
  Port 22
```
 
where the Host is the name you want, the User and the Hostname are the Jetson parameters (user@hostname). Now you have a SSH connection from your computer to the Nvidia Jetson. To enter the Nvidia Jetson terminal, just run the following command:
 
 ```bash
  ssh name
 ```
where ``name`` is the name you wrote in the config file.



## Commands to use the ROS remotely

### Jetson 

Open the terminal and run the following command

```bash

  export ROS_IP=`hostname -I | awk '{print $1}' `
  
```
Now you can launch the roscore

```bash
 roscore
```


### Computer


Open the terminal and run the following command, please replace the ``jetson_hostname`` by the hostname of your Jetson.

```bash

  export ROS_MASTER_URI='http://jetson_hostname.local.local:11311'
  
  export ROS_IP=`hostname -I | awk '{print $1}' `

```
You should be able to connect to the rosmaster created in the Nvidia Jetson, check if everything is working by running the followig command

```bash
  rostopic list
```
