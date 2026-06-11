# Utils

**Preliminaries**
```bash
export UR5e_2f_85_PATH=/home/asus-mivia/Desktop/UR-Control/UR5e-2f-85
# Start vnc-server
vncserver -geometry 1920x1080 -localhost no
``` 



# Usefull commands
```bash
docker exec -it <ID_OR_NAME> bash

# Clean docker build cache
docker builder prune --all
# Clean dandling image
docker image prune -f
```