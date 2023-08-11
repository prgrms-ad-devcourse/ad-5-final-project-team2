# How to run

Start docker container

```
docker run -it --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --privileged --runtime nvidia --gpus all --volume ${PWD}:/workspace/superslam --workdir /workspace --name superslam superslam:latest /bin/bash
```

Build superslam

```
cd superslam
sh build.sh
```

