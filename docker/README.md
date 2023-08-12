# How to run

Start docker container

```
docker run -it --env DISPLAY=$DISPLAY --env USER=$USER --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged --runtime nvidia --gpus all --volume ${PWD}/Dataset:/workspace/Dataset --workdir /workspace --name superslam superslam:latest /bin/bash
```

Build superslam

```
cd superslam
sh build.sh
```

